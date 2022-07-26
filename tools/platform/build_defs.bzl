def if_gpu(if_true, if_false = []):
    """Shorthand for select()'ing on whether we're building with gpu enabled
    Returns a select statement which evaluates to if_true if we're building
    with use_gpu enabled. Otherwise, the select statement evaluates to
    if_false.
    """
    return select({
        "//tools/platform:use_gpu": if_true,
        "//conditions:default": if_false,
    })

def copts_if_gpu():
    return if_gpu(["-DUSE_GPU=1"], ["-DUSE_GPU=0"])

def if_teleop(if_true, if_false = []):
    return select({
        "//tools/platform:with_teleop": if_true,
        "//conditions:default": if_false,
    })

def copts_if_teleop():
    return if_teleop(["-DWITH_TELEOP=1"], ["-DWITH_TELEOP=0"])

def if_x86_mode(if_true, if_false = []):
    return select({
        "//tools/platform:x86_mode": if_true,
        "//conditions:default": if_false,
    })

def if_aarch64_mode(if_true, if_false = []):
    return select({
        "//tools/platform:aarch64_mode": if_true,
        "//conditions:default": if_false,
    })
