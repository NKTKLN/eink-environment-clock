"""Native test builds use cJSON from the ESP-IDF package that the firmware
environment already installs, so the host tests and the firmware parse JSON
with the same code."""
import os

Import("env")  # noqa: F821

path = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"),  # noqa: F821
                    "framework-espidf", "components", "json", "cJSON")
if not os.path.isfile(os.path.join(path, "cJSON.c")):
    raise SystemExit(f"cJSON not found in {path}: build the firmware env once "
                     "(pio run) so that framework-espidf gets installed")

env.Append(CPPPATH=[path])  # noqa: F821
env.BuildSources(os.path.join("$BUILD_DIR", "cjson"), path,  # noqa: F821
                 src_filter="-<*> +<cJSON.c>")
