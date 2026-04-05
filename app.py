"""Legacy runtime entrypoint wrapper.

The primary runtime is ROS-first. This root entrypoint remains only as a thin
fallback wrapper around the legacy app implementation.
"""

from legacy_runtime.app_main import main


if __name__ == "__main__":
    main()
