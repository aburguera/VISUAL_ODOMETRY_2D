% Name        : odoData=compute_odometry(imgPath,firstFrame,lastFrame,desiredWidth)
% Description : Basic monocular 2D visual odometry. Assumes a bottom
%               looking camera at constant height. Output motions are
%               provided in pixels.
% Input       : imgPath - The path where the images are stored. The images
%                         must be named by a number and be png. Other
%                         nomenclatures and formats can be easily used by
%                         properly changing the get_image function.
%               firstFrame - The first image to use. For example, if the
%                         images in imgPath are named from 1.png to 100.png
%                         and only from 37.png to 98.png must be used,
%                         firstFrame must be 37.
%               lastFrame - The last image to use. In the previous example,
%                         lastFrame must be 98.
%               desiredWidth - Images are scaled to that width. The scaled
%                         height is computed to maintain the aspect ratio.
% Output      : odoData - Array of structures so that:
%                 * odoData(i).f: SIFT features within the corresp. image.
%                 * odoData(i).d: SIFT descriptors within the corr. image.
%                 * odoData(i).X: Motion (x,y,o)' from image corresponding
%                   to odoData(i-1) to image corresponding to odoData(i).
%                   odoData(0).X is set to (0,0,0)'.
%                 * odoData(i).imNum: the number (between firstFrame and
%                   lastFrame) of the image.
% Author      : Antoni Burguera Burguera
%               antoni.burguera@uib.es
% Note        : The motions assume that the forward direction is -Y axis in
%               the image.
% Note        : This code relies on the VLFeat library. It can be
%               downloaded at http://www.vlfeat.org/.
% Note        : Please, refer to the README.TXT file for information about
%               how to properly cite us if you use this software.
function odoData=compute_odometry(imgPath,firstFrame,lastFrame,desiredWidth)
    % Check if VLFeat is installed and active.
    if ~exist('vl_sift')
        odoData=[];
        disp('Please, install vl_feat. If already installed, go to the vlfeat toolbox folder and run vl_setup from matlab');
        return;
    end;
    % Prepare data for first image
    [odoData(1).f,odoData(1).d]=compute_features(get_image(imgPath,desiredWidth,firstFrame));
    odoData(1).X=zeros(3,1);
    odoData(1).imNum=firstFrame;
    print_progress(firstFrame,firstFrame,lastFrame);
    % Prepare data for the remaining frames
    for curFrame=firstFrame+1:lastFrame
        odoIndex=curFrame-firstFrame+1;
        % Get features and descriptors
        [odoData(odoIndex).f,odoData(odoIndex).d]=compute_features(get_image(imgPath,desiredWidth,curFrame));
        % Match with previous image
        [matches,~]=vl_ubcmatch(odoData(odoIndex-1).d,odoData(odoIndex).d);
        % Compute motion using RANSAC
        [odoData(odoIndex).X,fail]=ransac_estimate_motion(odoData(odoIndex-1).f(1:2,matches(1,:)),odoData(odoIndex).f(1:2,matches(2,:)),1000,5,20,.60);
        % If RANSAC fails, use the previous estimate as the current one.
        if fail
            odoData(odoIndex).X=odoData(odoIndex-1).X;
        end;
        odoData(odoIndex).imNum=curFrame;
        print_progress(curFrame,firstFrame,lastFrame);
    end;
return;

% Helper function to display progress
function print_progress(curFrame,firstFrame,lastFrame)
    curImage=curFrame-firstFrame+1;
    numImages=lastFrame-firstFrame+1;
    disp(sprintf('* IMAGE %d of %d. PCT: %3.1f%%',curImage,numImages,curImage*100/numImages));
return;

% Helper function to load an image. It assumes that images are named by a
% number and are png (for example 34.png). Change this function if image
% names are different.
function I=get_image(imgPath,desiredWidth,curFrame)
    I=imread([imgPath num2str(curFrame) '.png']);
    h=size(I,1);
    w=size(I,2);
    f=desiredWidth/w;
    I=im2single(rgb2gray(imresize(I,[h*f,desiredWidth])));
return;

% Helper function to compute features.
function [f,d]=compute_features(I)
    % Get features
    [f,d]=vl_sift(I);
    % Center them
    h=size(I,1);
    w=size(I,2);
    f(1,:)=f(1,:)-(w/2);
    f(2,:)=f(2,:)-(h/2);
    % Transform to x forward
    f(1:2,:)=-f(2:-1:1,:);
return;