function [ newRowVector,newColVector,modifiedImage ] = FindAllTargetCentroids( RGBImage, RGBtarget )

[ rowVector,colVector,modifiedImage ] = FindTargetCentroid( RGBImage, RGBtarget );

for i = 1:5
    [ row,col,modifiedImage ] = FindTargetCentroid( modifiedImage, RGBtarget );
    rowVector = [rowVector, row];
    colVector = [colVector, col];
end

[rowVector1,index1] = sort(rowVector);
colVectorHigh = colVector(index1);
[rowVector,index] = sort(rowVector(1:end),'descend');
colVectorLow = colVector(index);
newRowVector = [rowVector1(1),rowVector(1:end-1)];
newColVector = [colVectorHigh(1),colVectorLow(1:end-1)];

end
