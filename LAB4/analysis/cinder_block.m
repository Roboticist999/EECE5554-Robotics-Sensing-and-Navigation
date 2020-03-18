clear, close all; clc,
cinder_block1 = imread('gray_cinder_block01_rect.bmp');
cinder_block2 = imread('gray_cinder_block02_rect.bmp');
cinder_block3 = imread('gray_cinder_block03_rect.bmp');
cinder_block4 = imread('gray_cinder_block04_rect.bmp');
cinder_block5 = imread('gray_cinder_block05_rect.bmp');
cinder_block6 = imread('gray_cinder_block06_rect.bmp');

[y,x,harris_score] = harris(cinder_block1,'tile',[20 20],'thresh',100000);
points = cornerPoints([x,y]);
[features, points] = extractFeatures(cinder_block1, points);
figure,imshow(cinder_block1);hold on, plot(points)

% Initialize all the transforms to the identity matrix. Note that the
% projective transform is used here because the building images are fairly
% close to the camera. Had the scene been captured from a further distance,
% an affine transform would suffice.
% numImages = numel(buildingScene.Files);
image_group = {cinder_block1,cinder_block2,cinder_block3,cinder_block4,cinder_block5,cinder_block6};
numImages = numel(image_group);
tforms(numImages) = projective2d(eye(3));

% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);
imageSize(1,:) = size(cinder_block1);

% Iterate over remaining image pairs
for n = 2:numImages
    
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
%     I = readimage(buildingScene, n);
    
    % Convert image to grayscale.
%     grayImage = rgb2gray(I);    
    
    % Save image size.
    imageSize(n,:) = size(image_group{n});
    
    % Detect and extract SURF features for I(n).
    [y,x,harris_score] = harris(image_group{n},'tile',[20 20],'thresh',100000);
    points = cornerPoints([x,y]);
    [features, points] = extractFeatures(image_group{n},points);
%     figure,imshow(image_group{n});hold on, plot(points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true,'MatchThreshold',20,'MaxRatio',0.6);
       
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 4000,'MaxDistance',10);n
    
    % Compute T(n) * T(n-1) * ... * T(1)
    tforms(n).T = tforms(n).T * tforms(n-1).T;
end

% Compute the output limits  for each transform
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);   
end

avgXLim = mean(xlim, 2);

[~, idx] = sort(avgXLim);

centerIdx = floor((numel(tforms)+1)/2);

centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros(height,width,'like', cinder_block1);

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
%     I = readimage(buildingScene, i);   
   
    % Transform I into the panorama.
    warpedImage = imwarp(image_group{i}, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(image_group{i},1),size(image_group{i},2)), tforms(i), 'OutputView', panoramaView);i
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
    imshow(panorama)
end

figure
imshow(panorama)

for jj = 1:numImages
    camera_loc(jj,:) = [imageSize(jj,2)/2, imageSize(jj,1)/2, 1];
end

for ii = 1: numImages
    camera_loc(ii,:) = camera_loc(ii,:) * tforms(ii).T;
end
camera_loc(:,3) = 200;
figure
image('CData',panorama,'XData',xLimits,'YData',yLimits);
hold on,
plot3(camera_loc(:,1), camera_loc(:,2), camera_loc(:,3),'r*','MarkerSize',50)
set(gca, 'YDir','reverse'),grid on,axis equal, title('camera location with respect to ponorama');