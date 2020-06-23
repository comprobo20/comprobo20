deckHeight = 1;
x = linspace(0,1,5)';
x = [x; linspace(-1,0,5)'];
y = abs(x);

Zs = linspace(-2,2,50);
loftCurve = Zs.^2;

makeLoftedMesh(x, y, deckHeight, Zs, loftCurve)