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

sumOfDistances = 0;

GT_Matrix = imread('Ground_Truth.png');
GT_Matrix = imbinarize(GT_Matrix);        % Make sure the image is B/W
GTBlackPoints = blkPoints(GT_Matrix, 1000, 1000);
%scatter(GTBlackPoints(:,1), GTBlackPoints(:,2))


SLAM_Map_Matrix = imread('SLAM_Gen_Map.png');
SLAM_Map_Matrix = imbinarize(SLAM_Map_Matrix);        % Make sure the image is B/W
SLAMBlackPoints = blkPoints(SLAM_Map_Matrix, 1000, 1000);

[knearNeigh, distances] = knnsearch(GTBlackPoints, SLAMBlackPoints);

for actualDistance = 1:size(distances)
    sumOfDistances = sumOfDistances + distances(actualDistance, 1);
end

%% Generates matrix with x and y coordinates of each black point in the image
%     It allows a maximum of rowRes x colRes points

function blkMatrix = blkPoints(image, rowRes, colRes)
    [rowSize, colSize, depth] = size(image);
    blkMatrix = zeros(1,2);
    actualRow = 1;
    actualCol = 1;
    while actualRow <= rowSize
        while actualCol <= colSize
            isBlack = image(actualRow, actualCol, 1) + image(actualRow, actualCol, 2) + image(actualRow, actualCol, 3);
            if isBlack == 0
                blkMatrix = [blkMatrix; actualRow, actualCol];
            end
            actualCol = actualCol + fix(colSize/colRes);
        end
        actualCol = 1;
        actualRow = actualRow + fix(rowSize/rowRes);
    end
end
