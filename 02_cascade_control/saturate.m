function y = saturate(x, lower_limit, upper_limit)
    if x > upper_limit
        y = upper_limit;
    elseif x < lower_limit
        y = lower_limit;
    else
        y = x;
    end
end