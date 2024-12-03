using Pkg

Pkg.add([
    "COSMO",
    "CSV",
    "ColorSchemes",
    "ColorTypes",
    "Colors",
    "Contour",
    "ControlSystems",
    "DocStringExtensions",
    "FFMPEG",
    "FFTW",
    "GLM",
    "GLMakie",
    "HTTP",
    "ImageView",
    "Images",
    # "Ipopt",
    # "JuMP",
    "LoopVectorization",
    "Makie",
    "NetCDF",
    "ProgressBars",
    "ProgressLogging",
    "ProgressMeter",
    "RecipesBase",
    "Revise",
    "RobotDynamics",
    "RobotZoo",
    "Symbolics",
    "Test",
    "TickTock"
])

Pkg.precompile()
