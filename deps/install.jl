using Pkg

Pkg.add([
    "BenchmarkTools",
    "BlockDiagonals",
    "ComponentArrays",
    "ControlSystems",
    "ControlSystemsBase",
    "Convex",
    "CoordinateTransformations",
    "DataFrames",
    "Dates",
    "DifferentialEquations",
    "Distributions",
    "ECOS",
    "FiniteDiff",
    "ForwardDiff",
    "LinearAlgebra",
    "Interpolations",
    "GR",
    "GeometryBasics",
    "LoggingExtras",
    "Makie",
    "OSQP",
    "Random",
    "RecipesBase",
    "Rotations",
    "SparseArrays",
    "StaticArrays",
    "StatsBase",
    "Parameters",
    "Plots"
])

Pkg.precompile()