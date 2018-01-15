function [La,Pa,Ya] = POfinder(Aa,Ca)
% Positive Luenberger Observer Finder Using Theorem 2.1
    [Pa,Ya] = POLMIsolver(Aa,Ca);
    La = Pa\Ya;
end

