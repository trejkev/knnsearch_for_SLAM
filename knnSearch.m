%----------------------------------------------%
% Universidad de Costa Rica
% Engineering Faculty
% Electrical Engineering School
% Graduation project: 
% SLAM Algorithms comparison 
% using ROS nodes
%
% Created by: Kevin Trejos Vargas
% email: kevin.trejosvargas@ucr.ac.cr
%
% Description: This script takes two B/W 
%     images, gets their x and y coordinates
%     for each black point, and then compare
%     them using knnsearch algorithm, to give
%     a measurement of how similar they are.

%
% Instructions for use:
%     1. Load both images in the same directory,
%        one named 'Ground_Truth.png' and the
%        other named 'SLAM_Gen_Map.png'.
%     2. Run the script and wait for its results.
%
% Notes: 
%     1. Make sure both images come with same DPI.
%     2. Make sure both images have the same
%        alignment.
%----------------------------------------------%

%% Globals

if exist('Statistics.png', 'file') == 2
    delete('Statistics.png')
end

sumOfDistances = 0;

rowsSampler = 1000;
colsSampler = 1000;

% Make the ground truth suitable
GT_Matrix = imread('Ground_Truth.png');
GT_Matrix = imbinarize(GT_Matrix);
GTBlackPoints = blkPoints(GT_Matrix, rowsSampler, colsSampler);

% Make the SLAM map suitable
SLAM_Map_Matrix = imread('SLAM_Gen_Map.png');
SLAM_Map_Matrix = imbinarize(SLAM_Map_Matrix);
SLAMBlackPoints = blkPoints(SLAM_Map_Matrix, rowsSampler, colsSampler);

% Use the knnsearch function
[knearNeigh, distances] = knnsearch(GTBlackPoints, SLAMBlackPoints);

% Sum all the distances to get an idea of how different they are
[slamRowSize, slamColSize] = size(distances);
for actualDistance = 1:slamRowSize
    sumOfDistances = sumOfDistances + distances(actualDistance, 1);
end

f=figure('visible','off');
subplot(2,2,1)
histogram(distances(:,1))
title("Histogram for distances")
subplot(1,2,2)
boxplot(distances(:,1))
title("Boxplot for distances")
subplot(2,2,3)
text(0,0.96,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
text(0,0.95,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
text(0,0.8 ,"Mean: "               ); axis off
text(0.4,0.8, string(mean(distances(:,1))) ); axis off
text(0,0.7 ,"Sum: "                ); axis off
text(0.4,0.7, string(sumOfDistances)       ); axis off
text(0,0.6 ,"Samples: "             ); axis off
text(0.4,0.6, string(slamRowSize)          ); axis off
text(0,0.5 ,"Stdev: "              ); axis off
text(0.4,0.5, string(std(distances(:,1)))  ); axis off
text(0,0.4 ,"Min: "                ); axis off
text(0.4,0.4, string(min(distances(:,1)))  ); axis off
text(0,0.3 ,"Max: "                ); axis off
text(0.4,0.3, string(max(distances(:,1)))  ); axis off
text(0,0.2 ,"Mode: "               ); axis off
text(0.4,0.2, string(mode(distances(:,1))) ); axis off
text(0,0.16,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
text(0,0.15,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
title("Descriptive statistics")
saveas(f,'Statistics','png')

fprintf("Completed! \n")

%% Generates matrix with x and y coordinates of each black point in the image
%     It allows a maximum of rowRes x colRes points

function blkMatrix = blkPoints(image, rowRes, colRes)
    [rowSize, colSize, depth] = size(image);
    blkMatrix   = zeros(1,2);
    actualRow   = 1;
    actualCol   = 1;
    actualTrial = 1;
    while actualRow <= rowSize
        while actualCol <= colSize
            isBlack = image(actualRow, actualCol, 1) + image(actualRow, actualCol, 2) + image(actualRow, actualCol, 3);
            if isBlack == 0
                if actualTrial == 1
                    blkMatrix(1,1) = actualRow;
                    blkMatrix(1,2) = actualCol;
                else
                    blkMatrix = [blkMatrix; actualRow, actualCol];
                end
                actualTrial = actualTrial + 1;
            end
            actualCol = actualCol + fix(colSize/colRes);
        end
        actualCol = 1;
        actualRow = actualRow + fix(rowSize/rowRes);
    end
end
