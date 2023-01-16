#pragma once
/************************************************
 * @file CPUAffinity.h
 * @short contains CPU affinity definitions
 ***********************************************/

#include "sls/sls_detector_exceptions.h"
#include <iomanip>
#include <ios>
#include <numa.h>
#include <sched.h>
#include <variant>
#include <vector>

namespace sls {
namespace CPUAffinity {

class FixedCPUSet {
  public:
    FixedCPUSet() { zero(); }
    FixedCPUSet(const FixedCPUSet &o) { copy(o.cs); }
    FixedCPUSet(const cpu_set_t &o) { copy(o); }

    FixedCPUSet &operator=(const FixedCPUSet &o) { return copy(o.cs); }
    FixedCPUSet &operator=(const cpu_set_t &o) { return copy(o); }

    void zero() { CPU_ZERO(&cs); }

    void set(int cpu) { CPU_SET(cpu, &cs); }
    void clear(int cpu) { CPU_CLR(cpu, &cs); }

    bool is_set(int cpu) const { return CPU_ISSET(cpu, &cs); }
    int count() const { return CPU_COUNT(&cs); }

    FixedCPUSet &operator&=(const FixedCPUSet &o) {
        return apply([](auto r, auto a, auto b) { CPU_AND(r, a, b); }, o);
    }
    FixedCPUSet &operator|=(const FixedCPUSet &o) {
        return apply([](auto r, auto a, auto b) { CPU_OR(r, a, b); }, o);
    }
    FixedCPUSet &operator^=(const FixedCPUSet &o) {
        return apply([](auto r, auto a, auto b) { CPU_XOR(r, a, b); }, o);
    }

    void apply_to_task(pid_t task_id) const {
        size_t size = sizeof(cpu_set_t);
        int ret = sched_setaffinity(task_id, size, &cs);
        if (ret != 0)
            throw sls::RuntimeError("Error setting CPU affinity: " +
                                    std::string(strerror(errno)));
    }

    void apply_to_this_thread() const { apply_to_task(0); }

    static int get_max_nb_cpus() { return CPU_SETSIZE; }

    cpu_set_t &cpu_set() { return cs; }
    const cpu_set_t &cpu_set() const { return cs; }

    static FixedCPUSet all_ones()
    {
        FixedCPUSet c;
        const auto max_cpus = get_max_nb_cpus();
        for (int i = 0; i < max_cpus; ++i)
            c.set(i);
        return c;
    }

  private:
    FixedCPUSet &copy(const cpu_set_t &o) {
        memcpy(&cs, &o, sizeof(cpu_set_t));
        return *this;
    }

    template <class F> FixedCPUSet &apply(F f, const FixedCPUSet &o) {
        cpu_set_t res;
        f(&res, &cs, &o.cs);
        return copy(res);
    }
    cpu_set_t cs;
};

inline bool operator==(const FixedCPUSet &a, const FixedCPUSet &b) {
    return CPU_EQUAL(&a.cpu_set(), &b.cpu_set());
}

inline FixedCPUSet operator~(FixedCPUSet c)
{
    return c ^= FixedCPUSet::all_ones();
}


class ULongArrayBitSet {
  public:
    using ULong = unsigned long;
    static constexpr int ULongBits = sizeof(ULong) * 8;

    ULongArrayBitSet(int nb_bits) : mask((nb_bits - 1) / ULongBits + 1, 0) {}

    void zero() {
        for (auto &&m : mask)
            m = 0;
    }

    void set(int node) { mask.at(idx(node)) |= bit(node); }
    void clear(int node) { mask.at(idx(node)) &= ~bit(node); }
    bool is_set(int node) const {
        return (mask.at(idx(node)) & bit(node)) != 0;
    }

    int count() const {
        int c = 0;
        for (int i = 0; i < get_max_nb_bits(); ++i)
            if (is_set(i))
                ++c;
        return c;
    }

    int get_max_nb_bits() const { return mask.size() * ULongBits; }

    ULong &front() { return mask.front(); }
    const ULong &front() const { return mask.front(); }

  private:
    int idx(int node) const { return node / ULongBits; }
    ULong bit(int node) const { return 1UL << (node % ULongBits); }

    std::vector<ULong> mask;
};

class NUMAMask {
  public:
    using ULong = ULongArrayBitSet::ULong;
    using OSMask = std::pair<const ULong *, ULong>;

    NUMAMask() : mask(get_max_nb_nodes()) {}

    OSMask get_os_mask() const {
        if (count() > 0)
            return {&mask.front(), get_max_nb_nodes()};
        else
            return {nullptr, 0};
    };

    void zero() { mask.zero(); }
    void set(int node) { mask.set(node); }
    void clear(int node) { mask.clear(node); }
    bool is_set(int node) const { return is_set(node); }
    int count() const { return mask.count(); }

    static int get_max_nb_nodes() {
        if (numa_available() < 0)
            throw sls::RuntimeError("NUMA is not available");
        return numa_max_node() + 1;
    }

  private:
    ULongArrayBitSet mask;
};

inline std::ostream &operator<<(std::ostream &os, const NUMAMask &mask) {
    auto &&[node_mask, max_node] = mask.get_os_mask();
    constexpr int NbULongBits = sizeof(NUMAMask::ULong) * 8;
    int nb_longs = (max_node - 1) / NbULongBits + 1;
    os << std::hex << std::setfill('0');
    bool first = true;
    int word_bits = std::min(int(max_node), NbULongBits);
    auto begin = node_mask + nb_longs - 1, end = begin - nb_longs;
    for (auto it = begin; it != end; --it, first = false)
        os << (!first ? "," : "") << std::setw(word_bits / 4) << *it;
    return os << std::dec;
}

template <class CPUMask, class NUMAMask> struct CPUAffinityMask {
    CPUMask mask;

    CPUAffinityMask() = default;
    CPUAffinityMask(const CPUAffinityMask &o) = default;
    CPUAffinityMask(CPUAffinityMask &&o) = default;

    CPUAffinityMask &operator=(const CPUAffinityMask &o) = default;
    CPUAffinityMask &operator=(CPUAffinityMask &&o) = default;

    void zero() { mask.zero(); }
    void set(int cpu) { mask.set(cpu); }
    void clear(int cpu) { mask.clear(cpu); }
    bool is_set(int cpu) const { return mask.is_set(cpu); }
    int count() const { return mask.count(); }

    CPUAffinityMask &operator&=(const CPUAffinityMask &o) {
        mask &= o.mask;
        return *this;
    }
    CPUAffinityMask &operator|=(const CPUAffinityMask &o) {
        mask |= o.mask;
        return *this;
    }
    CPUAffinityMask &operator^=(const CPUAffinityMask &o) {
        mask ^= o.mask;
        return *this;
    }

    void apply_to_task(pid_t task_id) const { mask.apply_to_task(task_id); }
    void apply_to_this_thread() const { mask.apply_to_this_thread(); }

    int get_max_nb_cpus() const { return mask.get_max_nb_cpus(); }

    CPUMask &cpu_mask() { return mask; }
    const CPUMask &cpu_mask() const { return mask; }

    NUMAMask get_numa_mask() const {
        NUMAMask numa_mask;
        for (int i = 0; i < mask.get_max_nb_cpus(); ++i)
            if (mask.is_set(i))
                numa_mask.set(numa_node_of_cpu(i));
        return numa_mask;
    }
};

using FixedCPUSetAffinityMask = CPUAffinityMask<FixedCPUSet, NUMAMask>;

typedef std::vector<FixedCPUSetAffinityMask> FixedCPUSetAffinityList;

using AnyCPUAffinity = std::variant<std::monostate, FixedCPUSetAffinityMask>;

} // namespace CPUAffinity
} // namespace sls
