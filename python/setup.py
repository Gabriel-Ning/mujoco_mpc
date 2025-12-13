# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Install script for MuJoCo MPC."""

import os
import pathlib
import platform
import shutil
import setuptools
from setuptools.command import build_ext
from setuptools.command import build_py
from setuptools.command.develop import develop
from setuptools.command.egg_info import egg_info
import subprocess


Path = pathlib.Path


class GenerateProtoGrpcCommand(setuptools.Command):
  """Specialized setup command to handle proto compilation.

  Generates the `*_pb2{_grpc}.py` files from `*_proto`. Assumes that
  `grpc_tools.protoc` is installed.
  """

  description = "Generate `.proto` files to Python protobuf and gRPC files."
  user_options = []

  def initialize_options(self):
    self.build_lib = None

  def finalize_options(self):
    self.set_undefined_options("build_py", ("build_lib", "build_lib"))

  def run(self):
    """Generate protos into `*_pb2{_grpc}.py`."""
    # We import here because, if the import is at the top of this file, we
    # cannot resolve the dependencies without having `grpcio-tools` installed.
    from grpc_tools import protoc  # pylint: disable=import-outside-toplevel

    for proto_filename in ["agent.proto", "direct.proto", "filter.proto"]:
      self._generate_proto(protoc, proto_filename)

  def _generate_proto(self, protoc, proto_filename):
    proto_source_path = Path(
        "..", "mjpc", "grpc", proto_filename
    ).resolve()
    
    # Always generate into the source directory for editable installs and simplicity
    source_root = Path(__file__).parent.resolve()
    proto_module_relative_path = Path("mujoco_mpc", "proto", proto_filename)
    proto_destination_path = source_root / proto_module_relative_path
    
    proto_destination_path.parent.mkdir(parents=True, exist_ok=True)
    # Copy `proto_filename` into current source.
    shutil.copy(proto_source_path, proto_destination_path)

    # Output directory is the package root (so mujoco_mpc/proto/... structure is preserved)
    output_dir = source_root

    protoc_command_parts = [
        # We use `__file__`  as the first argument the same way as is done by
        # `protoc` when called as `__main__` here:
        # https://github.com/grpc/grpc/blob/21996c37842035661323c71b9e7040345f0915e2/tools/distrib/python/grpcio_tools/grpc_tools/protoc.py#L172-L173.
        __file__,
        f"-I{output_dir}",
        f"--python_out={output_dir}",
        f"--grpc_python_out={output_dir}",
        str(proto_destination_path),
    ]

    protoc_returncode = protoc.main(protoc_command_parts)

    if protoc_returncode != 0:
      raise subprocess.CalledProcessError(
          returncode=protoc_returncode,
          cmd=f"`protoc.main({protoc_command_parts})`",
      )

    self.spawn(["touch", str(proto_destination_path.parent / "__init__.py")])


class CopyBinariesCommand(setuptools.Command):
  """Specialized setup command to copy binaries next to python source.

  Assumes that the C++ gRPC binaries have been manually built and
  and located in the default `mujoco_mpc/build/bin` folder.
  """

  description = "Copy binaries next to python source."
  user_options = []

  def initialize_options(self):
    self.build_lib = None

  def finalize_options(self):
    self.set_undefined_options("build_py", ("build_lib", "build_lib"))

  def run(self):
    self._copy_binary("agent_server")
    self._copy_binary("ui_agent_server")
    self._copy_binary("direct_server")
    self._copy_binary("filter_server")

  def _copy_binary(self, binary_name):
    source_path = Path(f"../build/bin/{binary_name}")
    if not source_path.exists():
      # Try looking in the build directory relative to the workspace root if possible, 
      # but ../build/bin is the standard convention here.
      # If it fails, we just warn, because maybe we are just installing dependencies?
      # But for this specific repo, we expect them to exist.
      self.announce(f"WARNING: Cannot find `{binary_name}` binary from {source_path}. Skipping copy.")
      return

    # Copy to source directory
    source_root = Path(__file__).parent.resolve()
    destination_path = source_root / "mujoco_mpc" / "mjpc" / binary_name

    self.announce(f"{source_path.resolve()=}")
    self.announce(f"{destination_path.resolve()=}")

    destination_path.parent.mkdir(exist_ok=True, parents=True)
    shutil.copy(source_path, destination_path)


class CopyTaskAssetsCommand(setuptools.Command):
  """Copies task assets next to `mjpc/tasks`."""

  description = (
      "Copy task assets over to python source to make them accessible."
  )
  user_options = []

  def initialize_options(self):
    self.build_lib = None

  def finalize_options(self):
    self.set_undefined_options("build_ext", ("build_lib", "build_lib"))

  def run(self):
    mjpc_tasks_path = Path(__file__).parent.parent / "build" / "mjpc" / "tasks"
    if not mjpc_tasks_path.exists():
       self.announce("WARNING: Build MJPC tasks not found. Skipping asset copy.")
       return
    
    source_paths = (
      tuple(mjpc_tasks_path.rglob("*.xml"))
      + tuple(mjpc_tasks_path.rglob("*.png"))
      + tuple(mjpc_tasks_path.rglob("*.stl"))
      + tuple(mjpc_tasks_path.rglob("*.obj"))
    )
    relative_source_paths = tuple(p.relative_to(mjpc_tasks_path) for p in source_paths)
    
    # Copy to source directory
    source_root = Path(__file__).parent.resolve()
    destination_dir_path = source_root / "mujoco_mpc" / "mjpc" / "tasks"
    
    self.announce(
        f"Copying assets {relative_source_paths} from"
        f" {mjpc_tasks_path} over to {destination_dir_path}."
    )

    for source_path, relative_source_path in zip(source_paths, relative_source_paths):
      destination_path = destination_dir_path / relative_source_path
      destination_path.parent.mkdir(exist_ok=True, parents=True)
      shutil.copy(source_path, destination_path)


class BuildPyCommand(build_py.build_py):
  """Specialized Python builder to handle service dependencies.

  During build, this will generate the `*_pb2{_grpc}.py` files and copy
  binaries next to python source.
  """

  user_options = build_py.build_py.user_options

  def run(self):
    self.run_command("generate_proto_grpc")
    self.run_command("copy_task_assets")
    super().run()


class EggInfoCommand(egg_info):
  def run(self):
    self.run_command("copy_binaries")
    self.run_command("generate_proto_grpc")
    self.run_command("copy_task_assets")
    super().run()



class CMakeExtension(setuptools.Extension):
  """A Python extension that has been prebuilt by CMake.

  We do not want distutils to handle the build process for our extensions, so
  so we pass an empty list to the super constructor.
  """

  def __init__(self, name):
    super().__init__(name, sources=[])


class BuildCMakeExtension(build_ext.build_ext):
  """Uses CMake to build extensions."""

  def run(self):
    self._configure_and_build_binaries()
    self.run_command("copy_binaries")

  def _configure_and_build_binaries(self):
    """Check for CMake."""
    cmake_command = "cmake"
    build_cfg = "Release"
    mujoco_mpc_root = Path(__file__).parent.parent
    mujoco_mpc_build_dir = mujoco_mpc_root / "build"
    cmake_configure_args = [
        "-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE",
        f"-DCMAKE_BUILD_TYPE:STRING={build_cfg}",
        "-DBUILD_TESTING:BOOL=OFF",
        "-DMJPC_BUILD_GRPC_SERVICE:BOOL=ON",
    ]

    if platform.system() == "Darwin" and "ARCHFLAGS" in os.environ:
      osx_archs = []
      if "-arch x86_64" in os.environ["ARCHFLAGS"]:
        osx_archs.append("x86_64")
      if "-arch arm64" in os.environ["ARCHFLAGS"]:
        osx_archs.append("arm64")
      cmake_configure_args.append(f"-DCMAKE_OSX_ARCHITECTURES={';'.join(osx_archs)}")

    # TODO(hartikainen): We currently configure the builds into
    # `mujoco_mpc/build`. This should use `self.build_{temp,lib}` instead, to
    # isolate the Python builds from the C++ builds.
    print("Configuring CMake with the following arguments:")
    for arg in cmake_configure_args:
      print(f"  {arg}")
    subprocess.check_call(
        [
            cmake_command,
            *cmake_configure_args,
            f"-S{mujoco_mpc_root.resolve()}",
            f"-B{mujoco_mpc_build_dir.resolve()}",
        ],
        cwd=mujoco_mpc_root,
    )

    print("Building binaries with CMake")
    subprocess.check_call(
        [
            cmake_command,
            "--build",
            str(mujoco_mpc_build_dir.resolve()),
            "--target",
            "agent_server",
            "ui_agent_server",
            "direct_server",
            "filter_server",
            f"-j{os.cpu_count()}",
            "--config",
            build_cfg,
        ],
        cwd=mujoco_mpc_root,
    )


class DevelopCommand(develop):
  def run(self):
    self.run_command("build_ext")
    self.run_command("build_py")
    super().run()


setuptools.setup(
    name="mujoco_mpc",
    version="0.1.0",
    author="DeepMind",
    author_email="mujoco@deepmind.com",
    description="MuJoCo MPC (MJPC)",
    url="https://github.com/google-deepmind/mujoco_mpc",
    license="MIT",
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Natural Language :: English",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3 :: Only",
        "Topic :: Scientific/Engineering",
    ],
    packages=setuptools.find_packages(),
    python_requires=">=3.10",
    setup_requires=[
        "grpcio-tools",
        "grpcio",
    ],
    install_requires=[
        "brax",
        "grpcio",
        "matplotlib",
        "mediapy",
        "mujoco >= 3.1.1",
        "mujoco-mjx",
        "protobuf",
    ],
    extras_require={
        "test": [
            "absl-py",
        ],
    },
    ext_modules=[CMakeExtension("agent_server")],
    cmdclass={
        "build_py": BuildPyCommand,
        "build_ext": BuildCMakeExtension,
        "generate_proto_grpc": GenerateProtoGrpcCommand,
        "copy_binaries": CopyBinariesCommand,
        "copy_task_assets": CopyTaskAssetsCommand,
        "develop": DevelopCommand,
        "egg_info": EggInfoCommand,
    },
    package_data={
        "": [
            "mjpc/agent_server",
            "mjpc/ui_agent_server",
            "mjpc/direct_server",
            "mjpc/filter_server",
            "mjpc/tasks/**/*.xml",
        ],
    },
)
