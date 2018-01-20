function [ noisy ] = addNoise( orig, dist, param1, param2 )
%addNoise Adds noise specified by distribution and parameters
%   Detailed explanation goes here

sz = size(orig);

if isstruct(dist)
    dist_name = dist.dist;
    param1 = dist.param1;
    if isfield(dist,'param2')
        param2 = dist.param2;
    end
    if isfield(dist,'fixedrngseed') && dist.fixedrngseed
        seed = 5849774;
        rng(seed);
    else
        rng('shuffle');
    end
else
    dist_name = dist;
end

switch dist_name
    case {'Normal','Uniform','Rician'}
        noise = random(dist_name,param1,param2,sz);
    case {'Exponential','Poisson'}
        noise = random(dist_name,param1,sz);
    case {'Salt & Pepper'}
        noisy = imnoise(uint8(orig),dist_name,param1);
        if isfloat(orig)
            noisy = double(noisy);
        end
        return       
    case {'Normal Bimodal'}
        noise1 = random('Normal',-param1/2,param2,sz);
        noise2 = random('Normal',param1/2,param2,sz);
        idx = logical(random('Discrete Uniform',2,sz)-1);
        noise1(idx) = 0;
        noise2(~idx) = 0;
        noise = noise1 + noise2;
    otherwise
        error('Unknown noise model');
end


if isa(orig, 'uint8')
    noisy = double(orig) + noise;
    noisy = uint8(noisy);
elseif isfloat(orig)
    noisy = orig + noise;
    noisy = double(uint8(noisy));
else
    error('Input must be uint8 or double.');
end

end

