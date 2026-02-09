# pre-compiling script to remove strict compilcation flags from thirdparty files we have no control over

Import("env")

def middleware(node):
    p = str(node).replace("\\", "/")

    if "/third_party/" in p:
        new_flags = env.get("CCFLAGS", []) + ["-w"]
        return env.Object(
            node,
            CCFLAGS=new_flags
        )

    return None

env.AddBuildMiddleware(middleware)
