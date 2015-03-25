function [ output_args ] = recovercolor( input_args,outname,scale )

I = imread(input_args); %read image
I = imresize(I, scale); %resize image 

hight = floor(size(I,1)/3); %cut original image into three images
im1 = I(1:hight,:);
im2 = I(hight+1:2*hight,:);
im3 = I(2*hight+1:3*hight,:);

%ransac for image1 and image2
[x,y] = sift(im1, im2);
[R,T] = RANSAC(x(1:2,:),y(1:2,:),1000,3,size(x,2)*0.4);
%transform the image2 to the coordinate of image1
im2_t = uint8(zeros(size(im2)));
for y =1:size(im2_t,1)
    for x = 1:size(im2_t,2)
        V = R * ([x;y] + T);
        V = int16(V);
        if(V(1) > 0 && V(2) > 0 && V(1) <= size(im2,2) && V(2) <= size(im2,1))
            im2_t(y,x) = im2(V(2),V(1));
        end
    end
end
%ransac for image1 and image2
[x,y] = sift(im1, im3);
[R,T] = RANSAC(x(1:2,:),y(1:2,:),1000,3,size(x,2)*0.4);
%transform the image3 to the coordinate of image1
im3_t = uint8(zeros(size(im3)));
for y =1:size(im3_t,1)
    for x = 1:size(im3_t,2)
        V = R * ([x;y] + T);
        V = int16(V);
        if(V(1) > 0 && V(2) > 0 && V(1) <= size(im3,2) && V(2) <= size(im3,1))
            im3_t(y,x) = im3(V(2),V(1));
        end
    end
end
%perform brightness equalization
im1 = histeq(im1,imhist(im2_t));
im3_t = histeq(im3_t,imhist(im2_t));
%combine three color channels
my_image = zeros(size(im1,1),size(im1,2),3);
my_image(:,:,1)=uint8(im3_t);
my_image(:,:,2)=uint8(im2_t);
my_image(:,:,3)=uint8(im1);

my_image = uint8(my_image);
%cut nosiy edges
sidey = int16(0.08 * size(my_image,2));
sidex = int16(0.08 * size(my_image,1));
my_image = my_image(sidex:end-sidex,sidey:end-sidey,:);
%other arts
imshow(my_image);
imwrite(my_image,outname)
output_args = my_image;

end

