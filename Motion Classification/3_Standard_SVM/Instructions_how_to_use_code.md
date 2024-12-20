## Instructions for Traning and Testing the SVM Classifier:

Once the datasets are labeled, you can proceed with training and testing the standard SVM classifier by following these steps:

1. **Run `00_Creation_Dataset_with_GT_Labels`**  
   This Jupyter Notebook creates the training and test datasets for the standard SVM classifier.

2. **Run `01_Remove_Bias_Dataset_wGTLabel`**  
   This notebook removes bias from the training and test datasets for the standard SVM classifier.

3. **Run `02_Train_Standard_SVM_clf`**  
   This notebook trains the standard SVM classifier using a vector of 6 IMU signals (3x Accelerometer + 3x Gyroscope) with their respective labels.

4. **Run `03_Test_Standard_SVM_clf`**  
   This notebook tests the standard SVM classifier and provides the Confusion Matrices (CFM) and parameters such as accuracy, etc.

## Instructions for Getting the Label Predictions for the Navigation Solutions Datasets:

The datasets used for testing the navigation algorithms ZUPT-SVM and ZUPT-SVM-SHOE, where not labeled with ground truth labels. Therefore, such kind of datasets use a different flow.

1. **Run `00_Creation_Dataset_without_GT_Labels`**  
   This Jupyter Notebook creates the datasets for the predict the labels with the standard SVM classifier.

2. **Run `01_Remove_Bias_Dataset_woGTLabel`**  
   This notebook removes bias from the datasets for the predict the labels with the standard SVM classifier.

3. **Run `04_Label_Dataset_with_standard_SVM_Predictions`**  
   This notebook predicts the lables with the standard SVM classifier.

### Additional Notes

- Ensure that all necessary dependencies and libraries are installed before running the notebooks.
- Each notebook should be run in sequence to ensure the proper flow of data and processing.
- For any issues or more detailed instructions, refer to the comments within each notebook or contact the repository maintainer.
