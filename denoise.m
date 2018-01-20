function [ denoised, stats ] = denoise( img , params , noise_model)
%DENOISE computes argmin F(u,image) + alpha* R(u)
%   Detailed explanation goes here

% Print status messages?
if isfield(params, 'debugoutput') && params.debugoutput
    debugoutput = true;
    debugfzero = 'iter';
else
    params.debugoutput = false;
    debugoutput = false;
    debugfzero = 'notify';
end

% Number of pixels
n = length(img(:));

% Use discrepancy principle?
if isfield(params, 'alpha') && ~(isfield(params, 'discrep') && params.discrep)
    use_discrep = false;
    if ~isfield(params, 'std')
        params.std = NaN;
    end
else
    use_discrep = true;
end

% For bounds: use sample or exact quantiles
if isfield(params, 'sample_quantiles') && ~params.sample_quantiles
    sample_quantiles = false;
else
    sample_quantiles = true;
end

% For bounds: conflevel is not always required
if ~isfield(params, 'conflevel')
    params.conflevel = 0;
end

% Set fidelity specific parameters
switch params.fidelity
    case {'Bounds', 'Relaxed Bounds Residual'}
        params.alpha = 1;
        use_discrep = false;
        [lower, upper] = getBounds(img, noise_model, params.conflevel, sample_quantiles);
        params.lowerbounds = lower;
        params.upperbounds = upper;
    case {'Relaxed Bounds L1', 'Relaxed Bounds L2sq'}
        use_discrep = false;
        [lower, upper] = getBounds(img, noise_model, params.conflevel, sample_quantiles);
        params.lowerbounds = lower;
        params.upperbounds = upper;
    case {'Residual'}
        params.alpha = 1;
        use_discrep = false;
        params.delta = params.std * sqrt(n);
    case {'L1'}
        params.delta = params.std * n;
    case {'L2'}
        params.delta = params.std * sqrt(n);
    case {'L2 squared'}
        params.delta = params.std^2 * n;
    otherwise
        warning('Unknown fidelity.');
end

if ~use_discrep
    [denoised, stats] = do_minimization(img, params);
else
    % Discrepancy principle
    c = 1.1;
    delta = params.delta;
    assert(delta > 0);
    interval = [0 1e-4]; % Start search interval
    f = @(alpha) do_minimization(img, params, alpha, false) - c * delta; % The objective function

    % Adjust the search interval so that the function is positive at the right endpoint
    % and negative at the left endpoint (through search on a log scale)
    if debugoutput
        disp(['Start interval is: ' num2str(interval)]);
    end
    while f(interval(2)) < 0
        interval = [1 10] .* interval(2);
        if debugoutput
            disp(['Interval is now: ' num2str(interval)]);
        end
    end

    % Call the actual minimization
    options = optimset('TolX',1e-3 * interval(1),'Display',debugfzero); % Options for fzero
    params.alpha = fzero( f , interval , options ); % fzero finds a zero in the given interval
    [denoised, stats] = do_minimization(img, params); % Finally denoise using the optimal alpha returned by fzero
end

end