function balanceObjects()
spawnBox('balance',0,0,0.5,100,0.1,0.1,1);
sheetThickness = 0.1;
spawnBox('sheet',0,0,1 + sheetThickness/2,1,1,1,sheetThickness);
diskThickness = 0.1;
spawnDisk('weight1',0.25,0.25,1+diskThickness/2+sheetThickness,0.5,0.1,diskThickness);
spawnDisk('weight2',-0.25,-0.25,1+diskThickness/2+sheetThickness,0.5,0.1,diskThickness);
end