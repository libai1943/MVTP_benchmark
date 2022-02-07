function [solution, metrics] = SolveMVTP(mvtp)
[guesses, metrics, rub] = IdentifyHomotopyClass(mvtp);
if isempty(guesses)
    solution = [];
    return
end
[solution, metrics] = CalculateLocalOptimum(mvtp, guesses, metrics, rub);
end