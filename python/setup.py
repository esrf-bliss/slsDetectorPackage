"""
Setup file for slsdet
Build upon the pybind11 example found here: https://github.com/pybind/python_example
"""
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
import sys
import setuptools
import os

__version__ = os.environ.get('GIT_DESCRIBE_TAG', 'developer')


def get_conda_path():
    """
    Keep this a function if we need some fancier logic later
    """
    print('Prefix: ', os.environ['CONDA_PREFIX'])
    return os.environ['CONDA_PREFIX']


# class get_pybind_include(object):
#     """Helper class to determine the pybind11 include path
#     The purpose of this class is to postpone importing pybind11
#     until it is actually installed, so that the ``get_include()``
#     method can be invoked. """

#     def __init__(self, user=False):
#         self.user = user

#     def __str__(self):
#         import pybind11
#         return pybind11.get_include(self.user)


ext_modules = [
    Extension(
        '_slsdet',
        ['src/main.cpp',
        'src/enums.cpp',
        'src/detector.cpp',
        'src/network.cpp'],
        include_dirs=[
            # Path to pybind11 headers
            # get_pybind_include(),
            # get_pybind_include(user=True),
            os.path.join('../libs/pybind11/include'),
            os.path.join(get_conda_path(), 'include'),

        ],
        libraries=['SlsDetector', 'SlsReceiver', 'zmq'],
        library_dirs=[
            os.path.join(get_conda_path(), 'lib'),
            os.path.join(get_conda_path(), 'bin'),
        ],

        language='c++'
    ),
]


# As of Python 3.6, CCompiler has a `has_flag` method.
# cf http://bugs.python.org/issue26689
def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler.
    """
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.cpp') as f:
        f.write('int main (int argc, char **argv) { return 0; }')
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.distutils.errors.CompileError:
            return False
    return True


def cpp_flag(compiler):
    """Return the -std=c++[11/14] compiler flag.
    The c++14 is prefered over c++11 (when it is available).
    """
    if has_flag(compiler, '-std=c++14'):
        return '-std=c++14'
    elif has_flag(compiler, '-std=c++11'):
        return '-std=c++11'
    else:
        raise RuntimeError('Unsupported compiler -- at least C++11 support '
                           'is needed!')


class BuildExt(build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        'msvc': ['/EHsc'],
        'unix': [],
    }

    if sys.platform == 'darwin':
        c_opts['unix'] += ['-stdlib=libc++', '-mmacosx-version-min=10.7']

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        if ct == 'unix':
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append(cpp_flag(self.compiler))
            if has_flag(self.compiler, '-fvisibility=hidden'):
                opts.append('-fvisibility=hidden')
        elif ct == 'msvc':
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
        for ext in self.extensions:
            ext.extra_compile_args = opts

        print('**************************************************')
        print(ct)
        print(opts)
        print('**************************************************')
        build_ext.build_extensions(self)
        


def get_shared_lib():
    return [f for f in os.listdir('.') if '_slsdet' in f]

setup(
    name='slsdet',
    version=__version__,
    author='Erik Frojdh',
    author_email='erik.frojdh@psi.ch',
    url='https://github.com/slsdetectorgroup/slsDetectorPackage',
    description='Detector API for SLS Detector Group detectors',
    long_description='',
    packages=find_packages(exclude=['contrib', 'docs', 'tests']),
    ext_modules=ext_modules,
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)
