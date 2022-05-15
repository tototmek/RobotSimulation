X = 4 * [0 0.25 0.5 0.75 1 1.25 1.5 2]';
Y = [0.1 0.15 0.22 0.45 0.80 0.95 1 1]';
figure
plot(X, Y)
title("Dane wejściowe");

M = [ones(size(X, 1), 1) X X.^2 X.^3 X.^4 X.^5 X.^6];

W = M\Y;

x = linspace(0, max(X));
figure
plot(x, W(1) + W(2) * x + W(3) * x.^2 + W(4) * x.^3 + W(5) * x.^4 + W(6) * x.^5 + W(7) * x.^6)
title("Model");