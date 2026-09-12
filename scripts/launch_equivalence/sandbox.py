"""Linux containment for offline launch evaluation; no optional unsafe fallback."""

import ctypes
import errno
import os
import sys
from pathlib import Path


class UnsafeOperation(BaseException):
    """Escape ordinary launch exception handlers when an operation is forbidden."""


def contain(scratch: Path) -> int:
    """Restrict filesystem writes and deny process creation, networking and device control."""
    libc = ctypes.CDLL(None, use_errno=True)
    libc.syscall.restype = ctypes.c_long
    # Landlock syscall numbers are shared by the supported Linux architectures.
    create, add, restrict = 444, 445, 446
    abi = libc.syscall(create, 0, 0, 1)
    if abi < 0:
        error = ctypes.get_errno()
        reason = {
            errno.ENOSYS: "the kernel does not implement Landlock, or an outer sandbox hides its syscalls",
            errno.EOPNOTSUPP: "Landlock is disabled in the running kernel",
            errno.EPERM: "the syscall is denied, possibly by an outer container or sandbox",
        }.get(error, "the kernel rejected the Landlock ABI query")
        raise RuntimeError(
            f"Landlock unavailable: {errno.errorcode.get(error, error)} ({os.strerror(error)}); {reason}. "
            "Run on a host with Landlock enabled and its syscalls permitted. No launch files were evaluated."
        )
    if abi < 3:
        raise RuntimeError(
            f"Landlock ABI {abi} detected; ABI >= 3 is required for file truncation protection. "
            "Use a newer kernel with Landlock enabled. No launch files were evaluated."
        )

    class Ruleset(ctypes.Structure):
        _fields_ = [("handled_access_fs", ctypes.c_uint64)]

    class PathRule(ctypes.Structure):
        _pack_ = 1
        _fields_ = [("allowed_access", ctypes.c_uint64), ("parent_fd", ctypes.c_int32)]

    handled = (1 << 15) - 1
    ruleset = libc.syscall(create, ctypes.byref(Ruleset(handled)), ctypes.sizeof(Ruleset), 0)
    if ruleset < 0:
        raise OSError(ctypes.get_errno(), "landlock_create_ruleset")

    def allow(path, access):
        fd = os.open(path, getattr(os, "O_PATH", 0x200000) | os.O_CLOEXEC)
        try:
            rule = PathRule(access, fd)
            if libc.syscall(add, ruleset, 1, ctypes.byref(rule), 0):
                raise OSError(ctypes.get_errno(), f"landlock_add_rule: {path}")
        finally:
            os.close(fd)

    try:
        for path in Path("/").iterdir():
            if path.name == "dev" or path.is_symlink():
                continue
            allow(path, (1 << 2) | ((1 << 3) if path.is_dir() else 0))
        for path in ("/dev/null", "/dev/urandom"):
            allow(path, 1 << 2)
        allow(scratch, handled & ~(1 << 0))
        if libc.prctl(38, 1, 0, 0, 0) or libc.syscall(restrict, ruleset, 0):
            raise OSError(ctypes.get_errno(), "landlock_restrict_self")
    finally:
        os.close(ruleset)

    seccomp = ctypes.CDLL("libseccomp.so.2", use_errno=True)
    seccomp.seccomp_init.argtypes = [ctypes.c_uint32]
    seccomp.seccomp_init.restype = ctypes.c_void_p
    seccomp.seccomp_syscall_resolve_name.argtypes = [ctypes.c_char_p]
    seccomp.seccomp_rule_add.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint]
    seccomp.seccomp_load.argtypes = [ctypes.c_void_p]
    seccomp.seccomp_release.argtypes = [ctypes.c_void_p]
    ctx = seccomp.seccomp_init(0x7FFF0000)
    if not ctx:
        raise RuntimeError("seccomp_init failed")
    try:
        for name in (
            "execve",
            "execveat",
            "fork",
            "vfork",
            "clone",
            "clone3",
            "socket",
            "socketpair",
            "connect",
            "bind",
            "ioctl",
            "kill",
            "tkill",
            "tgkill",
            "ptrace",
            "process_vm_writev",
            "mount",
            "umount2",
            "unshare",
            "setns",
            "io_uring_setup",
            "bpf",
            "chmod",
            "fchmod",
            "fchmodat",
            "fchmodat2",
            "chown",
            "fchown",
            "lchown",
            "fchownat",
            "utime",
            "utimes",
            "futimesat",
            "utimensat",
            "setxattr",
            "lsetxattr",
            "fsetxattr",
            "removexattr",
            "lremovexattr",
            "fremovexattr",
        ):
            number = seccomp.seccomp_syscall_resolve_name(name.encode())
            # Python probes terminal capabilities while opening ordinary text streams.
            error = errno.ENOTTY if name == "ioctl" else errno.EPERM
            if number >= 0 and seccomp.seccomp_rule_add(ctx, 0x50000 | error, number, 0):
                raise RuntimeError(f"Could not restrict syscall {name}")
        if seccomp.seccomp_load(ctx):
            raise RuntimeError("seccomp_load failed")
    finally:
        seccomp.seccomp_release(ctx)

    def audit(event, args):
        if event in {"subprocess.Popen", "os.system", "os.fork", "os.posix_spawn", "socket.__new__"}:
            raise UnsafeOperation(f"Blocked during launch evaluation: {event}")

    sys.addaudithook(audit)
    return abi


if __name__ == "__main__":
    # The controller owns scratch cleanup; containment applies only to this subprocess.
    try:
        abi = contain(Path(sys.argv[1]))
    except Exception as exception:
        print(f"Sandbox check failed: {exception}", file=sys.stderr)
        raise SystemExit(2) from None
    print(f"Sandbox check passed: Landlock ABI {abi}; seccomp installed.")
