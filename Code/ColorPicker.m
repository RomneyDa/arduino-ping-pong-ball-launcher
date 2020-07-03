function RGBvector = ColorPicker( array )
%COLORPICKER pops up an image, takes the coordinates of a click, and
%returns the RGB color values at the location of the click.
%
%Dallin Romney

    image(array);                       %Display image
    axis image                          %Reproportion image

    [x, y]  = ginput(1);                %Optain the precise decimal coordinates of the user's click
    
    RGB = array(round(y), round(x), :); %Round the coordinates to the nearest integer and index out the pixel
    
    R = RGB(1, 1, 1);           %Index out each value from the third dimension - otherwise RGB will be a 1x1x3 array.
    G = RGB(1, 1, 2);
    B = RGB(1, 1, 3);
        
    RGBvector = [R, G, B];      %Concatenate the scalars to produce a 1x3 vector of RGB values
    
end

