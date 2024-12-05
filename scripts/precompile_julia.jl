println("STARTING PRECOMPILE IMPORTS")
using DifferentialEquations 
println("DONE PRECOMPILE IMPORTS")

f(u, p, t) = 1.01 * u
u0 = 1 / 2
tspan = (0.0, 1.0)
prob = ODEProblem(f, u0, tspan)
sol = solve(prob, Tsit5())


function f2!(d, u, p, t) 
    d .= 1.01 * u
end

u1 = [1.0, 2.0]
prob2 = ODEProblem(f2!, u1, tspan)
sol2 = solve(prob2, Tsit5())

println("DONE PRECOMPILING")