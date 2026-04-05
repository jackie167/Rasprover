from dataclasses import dataclass


@dataclass
class TimedMotionCommand:
    command: object
    expiry_time: float
    origin: str


@dataclass
class TimedCommand:
    command: object
    expiry_time: float
    origin: str


class MotionArbiter:
    """Keeps the latest motion command and invalidates it on timeout."""

    def __init__(self):
        self.active_command = None
        self.last_signature = None

    def set_command(self, command, now_seconds, origin='unknown'):
        timeout_ms = int(command.timeout_ms or 0)
        if timeout_ms <= 0:
            timeout_ms = 500
        expiry_time = now_seconds + (timeout_ms / 1000.0)
        self.active_command = TimedMotionCommand(command=command, expiry_time=expiry_time, origin=origin)
        self.last_signature = self.signature(command)
        return self.active_command

    def clear(self):
        self.active_command = None
        self.last_signature = None

    def expired(self, now_seconds):
        return self.active_command is not None and now_seconds >= self.active_command.expiry_time

    def has_active(self, now_seconds):
        return self.active_command is not None and not self.expired(now_seconds)

    def active_origin(self):
        return None if self.active_command is None else self.active_command.origin

    @staticmethod
    def signature(command):
        return (
            round(float(command.linear), 4),
            round(float(command.angular), 4),
            int(command.priority),
            str(command.mode),
            str(command.source),
        )


class TimedCommandArbiter:
    """Keeps the latest command of a lane and invalidates it on timeout."""

    def __init__(self, default_timeout_ms=500):
        self.default_timeout_ms = default_timeout_ms
        self.active_command = None
        self.last_signature = None

    def set_command(self, command, now_seconds, origin='unknown'):
        timeout_ms = int(getattr(command, 'timeout_ms', 0) or 0)
        if timeout_ms <= 0:
            timeout_ms = self.default_timeout_ms
        expiry_time = now_seconds + (timeout_ms / 1000.0)
        self.active_command = TimedCommand(command=command, expiry_time=expiry_time, origin=origin)
        self.last_signature = self.signature(command)
        return self.active_command

    def clear(self):
        self.active_command = None
        self.last_signature = None

    def expired(self, now_seconds):
        return self.active_command is not None and now_seconds >= self.active_command.expiry_time

    def has_active(self, now_seconds):
        return self.active_command is not None and not self.expired(now_seconds)

    def active_origin(self):
        return None if self.active_command is None else self.active_command.origin

    @staticmethod
    def signature(command):
        values = []
        for name in ('pan', 'tilt', 'speed', 'accel', 'mode', 'source', 'base_pwm', 'head_pwm'):
            if hasattr(command, name):
                value = getattr(command, name)
                if isinstance(value, float):
                    value = round(value, 4)
                values.append(value)
        return tuple(values)
