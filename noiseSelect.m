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

%% Noise select function
% Author: Chong WU
% Email: chongwu2-c@my.cityu.edu.hk

function [img,noiseType] = noiseSelect(image,number)

    switch number
        case 1
            img = image;
            noiseType = 'NoNoise';
        case 2
            img = imnoise(image,'gaussian',0,0.01);
            noiseType = 'gaussian_0.01';
        case 3
            img = imnoise(image,'gaussian',0,0.04);
            noiseType = 'gaussian_0.04';
        case 4
            img = imnoise(image,'gaussian',0,0.09);
            noiseType = 'gaussian_0.09';
        case 5
            img = imnoise(image,'speckle',0.01);
            noiseType = 'speckle_0.01';
        case 6
            img = imnoise(image,'speckle',0.04);
            noiseType = 'speckle_0.04';
        case 7
            img = imnoise(image,'speckle',0.09);
            noiseType = 'speckle_0.09';
        case 8
            img = imnoise(image,'salt & pepper',0.1);
            noiseType = 'salt_0.1';
        case 9
            img = imnoise(image,'salt & pepper',0.15);
            noiseType = 'salt_0.15';
        case 10
            img = imnoise(image,'salt & pepper',0.2);
            noiseType = 'salt_0.20';
        case 11
            img = imnoise(image,'salt & pepper',0.25);
            noiseType = 'salt_0.25';
        case 12
            img = imnoise(image,'salt & pepper',0.30);
            noiseType = 'salt_0.30';
        case 13
            H = fspecial('motion',20,45);
            img = imfilter(image,H,'replicate');
            noiseType = 'motion_20_45';
        case 14
            H = fspecial('motion',50,45);
            img = imfilter(image,H,'replicate');
            noiseType = 'motion_50_45';
        case 15
            H = fspecial('disk',30);
            img = imfilter(image,H,'replicate');
            noiseType = 'disk_30';

    end
        
end