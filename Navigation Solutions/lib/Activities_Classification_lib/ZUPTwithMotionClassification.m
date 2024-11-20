function [sigma_zupt, threshold_zupt] = ZUPTwithMotionClassification(classification_results)

    if 0 % Optimal Thresholds for: Subject I
        if classification_results == 0 % Stationary
            sigma_zupt = 0.005;
            threshold_zupt = 3;
        elseif classification_results == 1 || classification_results == 1+9 % Walking
            sigma_zupt = 0.02;
            threshold_zupt = 3; % 3
        elseif classification_results == 2 || classification_results == 2+9 % Fast walking
            sigma_zupt = 0.03;
            threshold_zupt = 4.6; % 4.6
        elseif classification_results == 3 || classification_results == 3+9 % Jogging
            sigma_zupt = 0.05;
            threshold_zupt = 5; %5 
        elseif classification_results == 4 || classification_results == 4+9 % Running
            sigma_zupt = 0.1;
            threshold_zupt = 6.7;
        elseif classification_results == 5 || classification_results == 5+9 % Sprinting
            sigma_zupt = 0.1;
            threshold_zupt = 7.8;
        elseif classification_results == 6 || classification_results == 6+9 % walking backward
            sigma_zupt = 0.03;
            threshold_zupt = 3.2;
        elseif classification_results == 7 || classification_results == 7+9 % running backward
            sigma_zupt = 0.03;
            threshold_zupt = 5;
        elseif classification_results == 8 || classification_results == 8+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 5.7;
        elseif classification_results == 9 || classification_results == 9+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 5.7;
        end
    end % End Subject I parameters

    if 0 % Optimal Thresholds for: Subject II
        if classification_results == 0 % Stationary
            sigma_zupt = 0.005;
            threshold_zupt = 3;
        elseif classification_results == 1 || classification_results == 1+9 % Walking
            sigma_zupt = 0.02;
            threshold_zupt = 4;
        elseif classification_results == 2 || classification_results == 2+9 % Fast walking
            sigma_zupt = 0.03;
            threshold_zupt = 5.2; % 4.5
        elseif classification_results == 3 || classification_results == 3+9 % Jogging
            sigma_zupt = 0.05;
            threshold_zupt = 5; % 5
        elseif classification_results == 4 || classification_results == 4+9 % Running
            sigma_zupt = 0.1;
            threshold_zupt = 7.8;
        elseif classification_results == 5 || classification_results == 5+9 % Sprinting
            sigma_zupt = 0.1;
            threshold_zupt = 8;
        elseif classification_results == 6 || classification_results == 6+9 % walking backward
            sigma_zupt = 0.03;
            threshold_zupt = 3.4;
        elseif classification_results == 7 || classification_results == 7+9 % running backward
            sigma_zupt = 0.03;
            threshold_zupt = 3.7;
        elseif classification_results == 8 || classification_results == 8+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 6;
        elseif classification_results == 9 || classification_results == 9+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 6;
        end
    end % End Subject II parameters
    

    if 0 % Optimal Thresholds for: Subject III 
        if classification_results == 0 % Stationary
            sigma_zupt = 0.005;
            threshold_zupt = 3;
        elseif classification_results == 1 || classification_results == 1+9 % Walking
            sigma_zupt = 0.02;
            threshold_zupt = 4.8;
        elseif classification_results == 2 || classification_results == 2+9 % Fast walking
            sigma_zupt = 0.03;
            threshold_zupt = 5.2;
        elseif classification_results == 3 || classification_results == 3+9 % Jogging
            sigma_zupt = 0.05;
            threshold_zupt = 6.9;
        elseif classification_results == 4 || classification_results == 4+9 % Running
            sigma_zupt = 0.1;
            threshold_zupt = 7.8;
        elseif classification_results == 5 || classification_results == 5+9 % Sprinting
            sigma_zupt = 0.1;
            threshold_zupt = 8.7;
        elseif classification_results == 6 || classification_results == 6+9 % walking backward
            sigma_zupt = 0.03;
            threshold_zupt = 4.5;
        elseif classification_results == 7 || classification_results == 7+9 % running backward
            sigma_zupt = 0.03;
            threshold_zupt = 6;
        elseif classification_results == 8 || classification_results == 8+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 6;
        elseif classification_results == 9 || classification_results == 9+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 6;
        end
    end % End Subject III parameters

    if 1 % Fixed for Subject I,II,III 
        if classification_results == 0 % Stationary
            sigma_zupt = 0.005;
            threshold_zupt = 3;
        elseif classification_results == 1 || classification_results == 1+9 % Walking
            sigma_zupt = 0.02;
            threshold_zupt = 4.8;
        elseif classification_results == 2 || classification_results == 2+9 % Fast walking
            sigma_zupt = 0.03;
            threshold_zupt = 5.2;
        elseif classification_results == 3 || classification_results == 3+9 % Jogging
            sigma_zupt = 0.05;
            threshold_zupt = 6.89;
        elseif classification_results == 4 || classification_results == 4+9 % Running
            sigma_zupt = 0.1;
            threshold_zupt = 7.8;
        elseif classification_results == 5 || classification_results == 5+9 % Sprinting
            sigma_zupt = 0.1;
            threshold_zupt = 8.7;
        elseif classification_results == 6 || classification_results == 6+9 % walking backward
            sigma_zupt = 0.03;
            threshold_zupt = 4.5;
        elseif classification_results == 7 || classification_results == 7+9 % running backward
            sigma_zupt = 0.03;
            threshold_zupt = 6;
        elseif classification_results == 8 || classification_results == 8+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 6;
        elseif classification_results == 9 || classification_results == 9+9 % Side stepping
            sigma_zupt = 0.08;
            threshold_zupt = 6;
        end
    end % End Subject III parameters
end