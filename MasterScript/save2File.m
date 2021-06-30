function [data] = save2File(rot1, rot2, trans1, trans2, error, time, sensorStat, frame, segment_name)
%sav2File Takes saved data as an arguement and saves it to a csv file as
%well as returns a table of the data

%INPUTS: 
%rot and trans are matrices containing rotational and translation
%data from the aurora device. error is the error of the measurment, time is
%the time at which the loop executes the data save, sensorStat is 2 if the
%reading is bad and 1 if it is good, frame is the frame number, segment
%name is the name of the file to save to 

%RETURNS: 
%   - data table of all of the data (meant for easy working in workspace)

%ADDITIONS: 
%   - Make filename a user-prompted question

    fprintf("\nIntiating file save procedure");
    fprintf("\nparsing data");
    Tx1 = trans1(:,1);
    Ty1 = trans1(:,2);
    Tz1 = trans1(:,3);
    
    %note that these are saved in terms of quaternions
    Qo1 = rot1(:,1);
    Qx1 = rot1(:,2);
    Qy1 = rot1(:,3);
    Qz1 = rot1(:,4);
    
    
    T = table(Tx1, Ty1, Tz1, Qo1, Qx1, Qy1, Qz1, error, time, sensorStat, frame);

    data = T;
    
    %here, let us use a try and catch statement to make sure that our data
    %save works
    
    
    try
        fprintf("\nattempting to write data");
        writetable(T, segment_name);
        
        
    catch
        fprintf("\nSomething went wrong... data was not saved. Trying again")
        writetable(T, segment_name);

    end 
    fprintf("\n**** Successfully written %s segment to a csv file ****", segment_name);
        
end

