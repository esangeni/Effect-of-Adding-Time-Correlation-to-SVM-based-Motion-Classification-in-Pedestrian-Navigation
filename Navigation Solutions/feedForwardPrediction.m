clc;
clear;
close all;

progress_bar = waitbar(0,'Initialization...');


currDir = pwd;

% datasetDir = [currDir, '\dataset','\2022_02_28'];
% datasetDir = [currDir, '\dataset','\2022_03_17'];
% datasetDir = [currDir, '\dataset','\2022_04_25'];
% datasetDir = [currDir, '\dataset','\2022_05_20'];
datasetDir = [currDir, '\dataset','\2022_06_02'];

% Journal
% datasetDir = [currDir, '\dataset','\2023_08_23'];


data_filename = 'exp100'; % without .mat %203

yaw_Deg =  0;

addpath([currDir, '\lib\google map lib']);
addpath([currDir, '\lib\INS lib']);
addpath([currDir, '\lib\Stance Phase Detector lib']);
addpath([currDir, '\lib\Activities_Classification_lib']);

addpath(datasetDir);
load([data_filename '_VN.mat']);

pyversion;
clf_filename = 'lib\Activities_Classification_lib\2022_03_17_&_04_25_clf_rbf_100_6sig.sav';
% clf_filename = [currDir,'\dataset\','2023_08_23\', '400HD_6sig_pca6_24000_balanced_clf.sav'];
% Filename is the name of the file.
svm_classifier = py.open(clf_filename,'rb');
svm_classifier = py.pickle.load(svm_classifier);

N = length(u);
svm_results = nan(9,N);
svm_results(1,:) = [1:N];
svm_results(2:4,:) = u(1:3,:);
svm_results(5:7,:) = u(4:6,:)/180*pi;
svm_results_temp = nan(1,N);
predict_temp = nan(1,2);
tic
for i = 20:20:N-1
    % SMV prediction
    if mod(i,1000) == 0
        waitbar(i/N,progress_bar,[num2str(i) '/' num2str(N) ' samples processed...'])
    end
    if i == 1000
        ini_time = toc;
        disp(['Estimated time: ' num2str(ini_time/i*N) ' s'])
    end
    predict_temp(:) = svm_classifier.predict([u( 1:3, i:i+1)' u(4:6, i:i+1)'/180*pi]); 
    
    svm_results_temp(i) = predict_temp(1);
end
close(progress_bar);
% svm_results(end+1) = svm_results_temp(2);
nanx = isnan(svm_results_temp);
t    = 1:numel(svm_results_temp);
svm_results_temp(nanx) = interp1(t(~nanx), svm_results_temp(~nanx), t(nanx),'nearest','extrap');

svm_results(9,:) = svm_results_temp;

if ~isfile([datasetDir '\' data_filename '_SVM.mat' ])
    save([datasetDir '\' data_filename '_SVM.mat' ],'svm_results')
    disp(['Prediction results saved to' datasetDir '\' data_filename '_SVM.mat' ])
end
