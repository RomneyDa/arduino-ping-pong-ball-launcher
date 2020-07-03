function [ centroidRow, centroidCol, newImage ] = FindTargetCentroid( array, target )
%FINDTARGETCENTROID returns the coordinates of the centroid of the first
% rectangle of a particular color in a picture (from the left), and the same
% picture with that rectange blackend out.
%
% Dallin Romney

    [numRows, numCols, ~] = size(array);                %Obtain the size of the array

    %Create a logical array that is true where the picture matches the RGB Vector completely
    equalsTarget = array(:, :, 1) == target(1) & array(:, :, 2) == target(2) & array(:, :, 3) == target(3);
    [rows, cols] = find(equalsTarget);                  %Index out the linear indices where the picure matches the target.

    firstRow = rows(1);                                 %For better readability
    firstCol = cols(1);

    for lastCol = firstCol:numCols - 1                  %From the first column that matches the color to the right edge of the picture
                                                              %(The minus-one ensures we don't go off the right edge of the picture)
        if equalsTarget(firstRow, lastCol + 1) == 0     %If the pixel to the right of the current pixel isn't the same color
            break;                                      %Then stop. You're ate the end of the rectangle.
        end  
    end
    
    for lastRow = firstRow:numRows - 1                  %From the first row that matches the color to the bottom edge of the picture
                                                              %(The minus-one ensures we don't go off the bottom edge of the picture)
        if equalsTarget(lastRow + 1, firstCol) == 0     %If the pixel under the current pixel isn't the same color
            break;                                      %Then stop. You're ate the end of the rectangle.
        end
    end
    
    centroidRow = (firstRow + lastRow) / 2;             %The centroid of a rectangle is halfway up it and halfway across it.
    centroidCol = (firstCol + lastCol) / 2;             %In other words, the average of two coordinates.
    
    newImage = array;                                   %Make a copy of the picture
    newImage(firstRow:lastRow, firstCol:lastCol, :) = 0;%Blacken the rectangular area
end

