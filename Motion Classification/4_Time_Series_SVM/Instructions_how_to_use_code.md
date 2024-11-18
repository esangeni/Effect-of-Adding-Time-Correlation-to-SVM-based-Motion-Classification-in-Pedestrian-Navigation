## Instructions for Traning and Testing the SVM Classifier:

Once the datasets are labeled, you can proceed with training and testing the Time Series SVM classifier by following these steps:

1. **Run `01_Creation_HD_Dataset_with_GT_Labels`**  
   This Jupyter Notebook creates the training and test high dimensional datasets for the Time Series SVM classifier.

2. **Run `02_Remove_Bias_HD_Dataset_wGTLabels`**  
   This notebook removes bias from the training and test datasets for the Time Series SVM classifier.

3. **Run `03_Train_Time_Series_SVM_clf`**  
   This notebook trains the Time Series SVM classifier using a window of time compressing vectors of 6 IMU signals (3x Accelerometer + 3x Gyroscope) with their respective labels.

4. **Run `03_Train_Time_Series_SVM_clf_Study_Time_Training`**  
   This notebook does an analysis of the computational time that takes the training of the Time Series SVM classifier using different windows of time and different amount of training data.

5. **Run `04_Test_Time_Series_SVM_clf`**  
   This notebook tests the Time Series SVM classifier and provides the Confusion Matrices (CFM) and parameters such as accuracy, etc.

## Instructions for Getting the Label Predictions for the Navigation Solutions Datasets:

The datasets used for testing the navigation algorithms ZUPT-SVM and ZUPT-SVM-SHOE, where not labeled with ground truth labels. Therefore, such kind of datasets use a different flow.

1. **Run `01_Creation_HD_Dataset_without_GT_Labels`**  
   This Jupyter Notebook creates the datasets for the predict the labels with the Time Series SVM classifier.

2. **Run `02_Remove_Bias_HD_Dataset_woGTLabels`**  
   This notebook removes bias from the datasets for the predict the labels with the Time Series SVM classifier.

3. **Run `04_Label_HD_Dataset_w_Time_Series_SVM_Predictions`**  
   This notebook predicts the lables with the Time Series SVM classifier.

### Additional Notes

- Ensure that all necessary dependencies and libraries are installed before running the notebooks.
- Each notebook should be run in sequence to ensure the proper flow of data and processing.
- For any issues or more detailed instructions, refer to the comments within each notebook or contact the repository maintainer.
