# TODO: figure out a way to properly do this... this code appends -w to everythign including src/ which is not good

# Import("env")

# def middleware(env_, node):
#     path = str(node).replace("\\", "/")

#     if "/third_party/" in path:
#         cloned = env_.Clone()
#         cloned.Append(CCFLAGS=["-w"])
#         return cloned

#     return env_

# env.AddBuildMiddleware(middleware)
