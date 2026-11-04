"""Host tests under AddressSanitizer + UndefinedBehaviorSanitizer.

Built with clang: its sanitizer runtime is linked statically, so the tests
do not depend on a libasan package being installed. The flags go to the
linker as well, which build_flags alone does not guarantee."""
Import("env")  # noqa: F821

FLAGS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all",
         "-fno-omit-frame-pointer", "-g", "-O1"]
env.Replace(CC="clang", CXX="clang++", LINK="clang")  # noqa: F821
env.Append(CCFLAGS=FLAGS, LINKFLAGS=FLAGS)  # noqa: F821
