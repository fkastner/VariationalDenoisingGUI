function [ lower, upper ] = getBounds( noisy, noise_model, beta, sample_quantiles )
%GETBOUNDS Constructs bounds according to the specified noise model
%   noisy                       The noisy image
%   noise_model                 A struct which specifies the noise
%       noise_model.dist           Name of the noise distribution
%       noise_model.param1         1st parameter of the distribution
%       noise_model.param2         2nd parameter of the distribution
%       noise_model.bg_noise       A sample of the noise (background noise)
%   beta                        "Confidence level"; beta/2 and 1-beta/2
%                               quantiles will be used for bound estimation. 
%                               Not used for uniform or s&p noise.
%   sample_quantiles            If true (default) uses the quantiles from 
%                               the noise sample in noise_model.bg_noise.
%                               If false uses the exact quantiles of the
%                               distribution

if nargin < 4
    sample_quantiles = true;
end

dist_name = noise_model.dist;

switch dist_name
    case 'Uniform'
        lower = noisy - noise_model.param2;
        upper = noisy - noise_model.param1;
    case 'Salt & Pepper'
        lower = noisy;
        lower(noisy == 255) = 0;
        upper = noisy;
        upper(noisy == 0) = 255;
    case 'Normal Bimodal'
        if sample_quantiles
            noise = noise_model.bg_noise(:);
            lower_quantile = quantile(noise, beta/2);
            upper_quantile = quantile(noise, 1-beta/2);
        else
            if beta == 0
                lower_quantile = -Inf;
                upper_quantile = Inf;
            else
                f = @(x) 0.5*cdf('Normal',x,-0.5*noise_model.param1,noise_model.param2)...
                       + 0.5*cdf('Normal',x, 0.5*noise_model.param1,noise_model.param2)...
                       - beta/2;
                lower_quantile = min(0, fzero(f,0) );
                upper_quantile = -lower_quantile;
            end
        end
        lower = noisy - upper_quantile;
        upper = noisy - lower_quantile;
    otherwise
        if sample_quantiles
            noise = noise_model.bg_noise(:);
            lower_quantile = quantile(noise, beta/2);
            upper_quantile = quantile(noise, 1-beta/2);
        else
            lower_quantile = icdf(dist_name, beta/2, noise_model.param1, noise_model.param2);
            upper_quantile = icdf(dist_name, 1-beta/2, noise_model.param1, noise_model.param2);
        end
        lower = noisy - upper_quantile;
        upper = noisy - lower_quantile;
end

lower = min( 255, max( 0, lower ) );
upper = min( 255, max( 0, upper ) );
lower(noisy == 0) = 0;
upper(noisy == 255) = 255;
% fprintf('Q_l: %f\tQ_u: %f\n',lower_quantile,upper_quantile); 
t = nnz( lower > upper );
if t > 0
    warning('Lower bounds are bigger than upper bounds: %i', t);
end

end
