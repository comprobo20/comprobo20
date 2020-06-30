function exportBoatForPrinting(scaleFactor, crossSection, Z, loftCurve, deckHeight, ballastLevel, averageInfill, avsFigToSave)
    % TODO: calculate avgInfill in this file (not passed in as an arg)
    % create the scaled STL
    layerThickness = 0.2;         % most prints use 0.2mm as the layer thickness
    printingTransform = zeros(4);
    printingTransform(4,4) = 1;
    printingTransform(1:3,1:3) = eul2rotm([pi 0 0]);
    printingTransform(2,4) = deckHeight*scaleFactor;
    secondTransform = zeros(4);
    secondTransform(4,4) = 1;
    secondTransform(1:3,1:3) = eul2rotm([0 0 pi/2]);
    printingTransform = secondTransform*printingTransform;
    scaledSTLFile = makeLoftedMesh(crossSection(:,1)*scaleFactor, crossSection(:,2)*scaleFactor, deckHeight*scaleFactor, Z*scaleFactor, loftCurve*scaleFactor, true, printingTransform);
    scaledSTLPath = fullfile(pwd,scaledSTLFile)
    if ~any(crossSection(:,2)<ballastLevel)
        newName = ['exportedBoat_noballast_avginfill_',num2str(averageInfill),'.stl'];
    else
        newName = ['exportedBoat_avginfill_',num2str(averageInfill),'.stl'];
        f2 = figure;
        % rotate the points to be consistent with the orientation of the 3d print
        crossSectionRotated = crossSection*[cosd(180) -sind(180); sind(180) cosd(180)]';
        crossSectionRotated(:,2) = crossSectionRotated(:,2)+deckHeight;
        % plot the edges
        plot(scaleFactor*crossSectionRotated(:,1), scaleFactor*crossSectionRotated(:,2),'k');
        hold on;
        % draw a line for the ballast level
        flippedBallastLevel = deckHeight - ballastLevel;
        plot(xlim(),scaleFactor*[flippedBallastLevel flippedBallastLevel],'k');
        ylim([min(min(ylim),-(max(ylim)-min(ylim))*0.1),max(ylim)]);
        axis equal;
        xlabel('x (mm)');
        ylabel('z (mm)');
        ylims = ylim;
        xlims = xlim;

        deckToBallastArrow = annotation('doublearrow');
        deckToBallastArrow.Parent = gca;
        xrange = max(xlims)-min(xlims);
        deckToBallastArrow.Position = [xrange*0.75+xlims(1), 0, 0, scaleFactor*flippedBallastLevel];
        % modify diagram so that the transition of the infill happens on a
        % multiple of 0.2mm
        text(xrange*0.77+xlims(1), scaleFactor*flippedBallastLevel*0.5, ['20% infill',char(10),num2str(round(1/layerThickness*(deckHeight-ballastLevel)*scaleFactor)*layerThickness),'mm']);

        disp('Warning this does not respect density ratio other than default');

        ballastToBottomArrow = annotation('doublearrow');
        ballastToBottomArrow.Parent = gca;
        ballastToBottomArrow.Position = [xrange*0.75+xlims(1), scaleFactor*flippedBallastLevel, 0, scaleFactor*(max(crossSectionRotated(:,2))-flippedBallastLevel)];

        text(xrange*0.77+xlims(1), 0.5*scaleFactor*flippedBallastLevel+0.5*scaleFactor*max(crossSectionRotated(:,2)), '100% infill');
        printingInstructionsPDF = [newName(1:end-length('stl')),'pdf'];
        printPlateVisual = area([xlims(1) xlims(2) xlims(2) xlims(1) xlims(1)],[ylims(1) ylims(1) 0 0 ylims(1)],'facecolor',[0.9, 0.9, 0.9],'LineStyle','none');
        alpha(printPlateVisual,0.5);
        text(-0.05*xrange, ylims(1)/2, 'Print Plate');
        title('x-z cross section (y = 0)');
        saveas(f2,printingInstructionsPDF);
        disp(['Exported ',printingInstructionsPDF]);
        if nargin >= 8
            avsPredictions = [newName(1:end-length('stl')-1),'_avs.pdf'];
            saveas(avsFigToSave,avsPredictions);
            disp(['Exported ',avsPredictions]);
        end
    end
    copyfile(scaledSTLPath, newName);
    disp(['Exported ',newName]);
end