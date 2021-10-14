%% Improved superpixel-based fast fuzzy C-means clustering
%
% Copyright (c) 2019, Chong WU
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% * Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
% * Redistributions in binary form must reproduce the above copyright notice,
%   this list of conditions and the following disclaimer in the documentation
%   and/or other materials provided with the distribution.
%   
% * Neither the name of City University of Hong Kong nor the names of its
%   contributors may be used to endorse or promote products derived from this
%   software without specific prior written permission.
%   
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

% Author: Chong WU
% Email: chongwu2-c@my.cityu.edu.hk
% If you use this code please cite:
% C. Wu, L. Zhang, H. Zhang and H. Yan, "Improved Superpixel-Based Fast Fuzzy C-Means Clustering for Image Segmentation," 2019 IEEE International Conference on Image Processing (ICIP), 2019, pp. 1455-1459, doi: 10.1109/ICIP.2019.8803039.
%% _______________________________________________________

clear, clc
close all
addpath('/FuzzySLIC');
addpath('/SFFCM');

for k = 1:10
    path = 'toy_example_image';
    fds = fileDatastore(fullfile('toy_example_groundtruth'),'ReadFcn',@load,'FileExtensions','.mat');
    imds = imageDatastore(path);
    numOfImg = 10;
    
    sumISFFCMri = 0;
    sumISFFCMvi = 0;
    sumISFFCMgce = 0;
    sumISFFCMbde = 0;
    ISFFCMt = 0;

    
    for i = 1:numOfImg
        path1 = imds.Files{i};
        img = imread(path1);
        [img,noiseType] = noiseSelect(img,k);
        groundTruth = read(fds);
        groundTruth = groundTruth.groundTruth;

        cluster=2;
     
        tic
        [L1,numlabels] = FuzzySLIC(img,60,17);

        L1 = L1+1;
        L2 = L1;
        L2 = imdilate(L1,strel('square',2));

        [~,~,Num,centerLab]=Label_image(img,L2);

        Label=w_super_fcm(L2,centerLab,Num,cluster);
        ISFFCMt = ISFFCMt + toc;
               
        for j = 1:length(groundTruth)

            GT = groundTruth{1,j}.Segmentation;

            [ri,gce,vi]=compare_segmentations(Label,GT);
            bde = compare_image_boundary_error(double(GT), double(Label));  

            sumISFFCMri = sumISFFCMri+ri/length(groundTruth);
            sumISFFCMgce =sumISFFCMgce+gce/length(groundTruth);
            sumISFFCMvi = sumISFFCMvi+vi/length(groundTruth);
            sumISFFCMbde = sumISFFCMbde+bde/length(groundTruth);
      
        end

        disp('Current err for ISFFCM:  BDE  RI  VOI  GCE Time:');
        disp([num2str(sumISFFCMbde/i) '   ' num2str(sumISFFCMri/i) ...
             '   ' num2str(sumISFFCMvi/i) '   ' num2str(sumISFFCMgce/i) '   ' num2str(ISFFCMt/i)]);
  
    end
    
end