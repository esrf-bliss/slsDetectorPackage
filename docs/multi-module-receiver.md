# Multi-Module Receiver Proposal

## Introduction
The `slsReceiver` is currently designed to process data coming from a single detector module. In the case of the Eiger, each `slsReceiver` instance reads UDP packets from a half-module. A `Listener` instance is created for each UDP port the receiver listens to, and a dedicated thread performs system calls to retrieve the data from a single detector (half) module. Such implementation implies a high number of threads from large detectors, which might compete for system resources and degrade overall performance.

## Multi-Module Receiver
Aggregating data from multiple modules into a single `slsReceiver` with a single `Listener` could not only simplify the detector configuration but it could also improve DAQ performance.

The following two options can be envisaged for the implementation.

### Multi-Port Receiver
In this approch the `Listener` reads packets from different UDP ports inside a single `Listener` thread. Even if `select` could be used to wait for data, just reading sequentially from all the ports might give a better result given the high event rate of the detectors.

This option allows balancing the network traffic to a single `slsReceiver` over multiple hardware links, allowing the control of big detectors by 10/25 Gbit Etherner adapters.

### Single-Port Receiver
This solution configures all the modules to send data to a single UDP port. In order to keep performance both at packet reception (zero memory copy) and at frame assembly (SIMD instructions in Eiger 4-bit mode), this solution could require an additional indexing layer in the `PacketBlock` class.

The main advantage of this option is the simplification of the detector configuration. The main drawback is that it forces that all the traffic towards a `slsReceiver` passes through a single NIC.

### Parametrization
In a system with `M={Mx,My}` horizontal and vertical detector modules, given `O={Ox,Oy}` the number of horizontal and vertical modules per receiver, the total number of `slsReceiver` instances will be `(Mx * My) / (Ox * Oy)`. In the limit case where `O == M`, a single `slsReceiver/Listener` will handle all the detector data.

For the sake of simplicity, `Mx` and `My` must be multiples of `Ox` and `Oy`, respectively: `M % O == {0,0}`.

## Configuration file
A new parameter in the configuration file will be needed to specify the number of modules per receiver `O`, which defaults to {1,1}:

```
recvmods <Ox> <Oy>
```

The detector parameters affecting the receiver configuration might suffer an optimization in the case where the `Listener` reads data from a single port.

## Detector client API
Likewise, a new function in the Detector API will allow setting `O`:

``` c++
void setModulesPerReceiver(defs::xy num_recv_mods);
```

## Receiver client API
The above function will directly configure `O` in the `slsReceiver` instances by means of the equivalent function in the receiver client API:

``` c++
void setModulesPerReceiver(defs::xy num_recv_mods);
```

with `Command identifier: F_SET_RECEIVER_MODULES`

