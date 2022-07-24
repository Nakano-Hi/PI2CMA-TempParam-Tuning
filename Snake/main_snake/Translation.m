

function translate = Translation(angles, ForWhat)
    if strcmp(ForWhat,'Robot')
        translate = round((angles + pi)*4096/(2*pi));
    elseif strcmp(ForWhat,'Vrep') 
        translate = angles;
    else
        error('Sorry, the application is not implemented')
    end
end