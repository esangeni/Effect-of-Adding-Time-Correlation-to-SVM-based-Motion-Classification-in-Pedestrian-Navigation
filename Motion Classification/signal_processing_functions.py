# Libraries:
import math
import numpy as np
import pandas as pd
import scipy.io as spio
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# convert deg to radians
def deg_to_rad(dr):
    return (dr*math.pi)/180

# convert the .mat datsets to pandas datframe
def mat2pandas(file_folder, file_name):
    mat = spio.loadmat(file_folder + file_name +'.mat');
    mdata = mat['u'];  # variable in mat file
    mdata = np.transpose(mdata);
    # creating df object with columns specified    
    df = pd.DataFrame(mdata, columns = ['acc1','acc2','acc3',
                                        'gyro1','gyro2','gyro3',
                                        'na','timestamp','na2',
                                        'airPressure','samplePeriodS','magnetometer_x',
                                        'magnetometer_y','magnetometer_z','temperatureC',]);
    df = df.drop(['na', 'na2','airPressure','samplePeriodS',
                  'magnetometer_x','magnetometer_y','magnetometer_z','temperatureC'], axis=1);
    # reorganize dataset to timestamp be the first column
    df = df[['timestamp','acc1','acc2','acc3','gyro1','gyro2','gyro3']]
    # convert from deg/min -> rad/min
    df['gyro1'] = deg_to_rad(df['gyro1']);
    df['gyro2'] = deg_to_rad(df['gyro2']);
    df['gyro3'] = deg_to_rad(df['gyro3']);
    return df

# Signal processing --> Magnitude acc & gyro:   
def magnitude(df,window):     
    df['acc']  = np.abs(np.sqrt(df['acc1']**2 + df['acc2']**2 + df['acc3']**2))
    df['gyro'] = np.abs(np.sqrt(df['gyro1']**2 + df['gyro2']**2 + df['gyro3']**2))
    df = df.drop(['acc1','acc2','acc3','gyro1','gyro2','gyro3'], axis=1) 

    df = df.div(window).rolling(window).sum();
    df = df.dropna().reset_index(drop=True); 
    return df

# Select the important maximums and minimums:
def select_max_min(sp_df,PP_df,NP_df):
    df_PP_peaks = pd.DataFrame();
    df_NP_peaks = pd.DataFrame();
    df_PP_peaks['id'] = PP_df; df_NP_peaks['id'] = NP_df;
    df_PP_peaks['timestamp'] = sp_df['timestamp'].loc[PP_df].to_numpy();
    df_NP_peaks['timestamp'] = sp_df['timestamp'].loc[NP_df].to_numpy();
    df_PP_peaks['label'] = 'PP'; df_NP_peaks['label'] = 'NP';
    df_peaks = pd.concat([df_PP_peaks, df_NP_peaks], ignore_index = True);
    df_peaks = df_peaks.sort_values('id').reset_index(drop=True);

    df_peaks_ = df_peaks
    for i in range(1,len(df_peaks)-1):
      if df_peaks['label'].loc[i-1] == 'PP' and df_peaks['label'].loc[i] == 'PP' and df_peaks['label'].loc[i+1] == 'PP':
          df_peaks_ = df_peaks_.drop(i)
    return df_peaks_

# Label the activity and the ZUPT
def label(df,df_peaks,zupt_error,activity):
    df['label'] = activity
    init_val_df = df['gyro'].loc[0]
    final_val_df = df['gyro'].iloc[-1]

    for i in range(1,len(df_peaks)-1):

      # initial tram no walking
      if df_peaks['label'].iloc[i] == 'PP' and i == 1:
        index_1st_peak = df_peaks['id'].iloc[i]
        for j in range(0,index_1st_peak):
          if df['gyro'].iloc[j] <= init_val_df*zupt_error:
            df['label'].iloc[j] = 'ZUPT';

      # tram between gaits
      if df_peaks['label'].iloc[i-1] == 'PP' and df_peaks['label'].iloc[i] == 'NP' and df_peaks['label'].iloc[i+1] == 'PP':
        index_max1 = df_peaks['id'].iloc[i-1]
        value_max1 = df['gyro'].iloc[df_peaks['id'].iloc[i-1]]
        index_min = df_peaks['id'].iloc[i]
        value_min = df['gyro'].iloc[df_peaks['id'].iloc[i]]
        index_max2 = df_peaks['id'].iloc[i+1]
        value_max2 = df['gyro'].iloc[df_peaks['id'].iloc[i+1]]
        for j in range(index_max1,index_max2):
          if df['gyro'].iloc[j] <= value_min*zupt_error:
            df['label'].iloc[j] = 'ZUPT_'+activity

      # final tram no walking
      if df_peaks['label'].iloc[i] == 'PP' and i == len(df_peaks)-2:
        index_n_peak = df_peaks['id'].iloc[i]
        for j in range(index_n_peak, len(df['gyro'])):
          if df['gyro'].iloc[j] <= final_val_df*zupt_error:
            df['label'].iloc[j] = 'ZUPT'
      
    return df

def post_processing_labeling(df,df_peaks,activity):
    for i in range(1,len(df_peaks)-1):
        # tram between gaits
        if df_peaks['label'].iloc[i-1] == 'PP' and df_peaks['label'].iloc[i] == 'NP' and df_peaks['label'].iloc[i+1] == 'PP':
          index_max1 = df_peaks['id'].iloc[i-1]
          index_max2 = df_peaks['id'].iloc[i+1]
          index_min =  df_peaks['id'].iloc[i]
          #print(index_max1,index_max2)
          for j in range(index_max1,index_min):
            if df['label'].loc[j] == 'ZUPT_' + activity:
              index_first_zupt = j;
              break;
          for j in reversed(range(index_min,index_max2)):
            if df['label'].loc[j] == 'ZUPT_' + activity:
              index_last_zupt = j;
              break;
          for j in range(index_first_zupt,index_last_zupt):
              df['label'].loc[j] = 'ZUPT_' + activity;

    return df

# Plot 8 graphics
def plot8(df1,df2,df3,df4,x_lim_low_exp_1,x_lim_upp_exp_1,x_lim_low_exp_2,x_lim_upp_exp_2,window):
    fig = plt.figure(figsize=(15, 15)); #dpi -- resolution graphic 100 is standard
    gs = GridSpec(nrows=4, ncols=2)
    ax0 = fig.add_subplot(gs[0, 0]); ax0.set_xlim([x_lim_low_exp_1, x_lim_upp_exp_1]);
    ax1 = fig.add_subplot(gs[0, 1]); ax1.set_xlim([x_lim_low_exp_2, x_lim_upp_exp_2]);
    ax2 = fig.add_subplot(gs[1, 0]); ax2.set_xlim([x_lim_low_exp_1, x_lim_upp_exp_1]);
    ax3 = fig.add_subplot(gs[1, 1]); ax3.set_xlim([x_lim_low_exp_2, x_lim_upp_exp_2]);
    ax4 = fig.add_subplot(gs[2, 0]); ax4.set_xlim([x_lim_low_exp_1, x_lim_upp_exp_1]);
    ax5 = fig.add_subplot(gs[2, 1]); ax5.set_xlim([x_lim_low_exp_2, x_lim_upp_exp_2]);
    ax6 = fig.add_subplot(gs[3, 0]); ax6.set_xlim([x_lim_low_exp_1, x_lim_upp_exp_1]);
    ax7 = fig.add_subplot(gs[3, 1]); ax7.set_xlim([x_lim_low_exp_2, x_lim_upp_exp_2]);
    ax0.plot(df1['timestamp'], df1[['acc1']], 'b', label='acc1')
    ax0.plot(df1['timestamp'], df1[['acc2']], 'r', label='acc2')
    ax0.plot(df1['timestamp'], df1[['acc3']], 'g', label='acc3')
    ax1.plot(df2['timestamp'], df2[['acc1']], 'b', label='acc1')
    ax1.plot(df2['timestamp'], df2[['acc2']], 'r', label='acc2')
    ax1.plot(df2['timestamp'], df2[['acc3']], 'g', label='acc3')
    ax2.plot(df3['timestamp'], df3['acc'].div(window).rolling(window).sum());
    ax3.plot(df4['timestamp'], df4['acc'].div(window).rolling(window).sum());
    ax4.plot(df1['timestamp'], df1[['gyro1']], 'b', label='gyro1')
    ax4.plot(df1['timestamp'], df1[['gyro2']], 'r', label='gyro2')
    ax4.plot(df1['timestamp'], df1[['gyro3']], 'g', label='gyro3')
    ax5.plot(df2['timestamp'], df2[['gyro1']], 'b', label='gyro1')
    ax5.plot(df2['timestamp'], df2[['gyro2']], 'r', label='gyro2')
    ax5.plot(df2['timestamp'], df2[['gyro3']], 'g', label='gyro3')
    ax6.plot(df3['timestamp'], df3['gyro'].div(window).rolling(window).sum());
    ax7.plot(df4['timestamp'], df4['gyro'].div(window).rolling(window).sum());
    ax0.set_title('Acc components -> Walking 60 step/min', fontweight="bold");
    ax1.set_title('Acc components -> Sprinting 180 step/min', fontweight="bold");
    ax2.set_title('Magnitude acceleration -> Walking 60 step/min', fontweight="bold");
    ax3.set_title('Magnitude acceleration -> Sprinting 180 step/min', fontweight="bold");
    ax4.set_title('Gyro components -> Walking 60 step/min', fontweight="bold");
    ax5.set_title('Gyro components -> Sprinting 180 step/min', fontweight="bold");
    ax6.set_title('Magnitude angular velocity -> Walking 60 step/min', fontweight="bold");
    ax7.set_title('Magnitude angular velocity -> Sprinting 180 step/min', fontweight="bold");
    return fig

def plot_PP(sp_100_df,sp_300_df,sp_500_df,sp_700_df,sp_900_df,sp_1100_df,sp_1300_df,sp_1500_df,sp_1700_df,
            PP_sp_100_df,PP_sp_300_df,PP_sp_500_df,PP_sp_700_df,PP_sp_900_df,PP_sp_1100_df,PP_sp_1300_df,PP_sp_1500_df,PP_sp_1700_df):
    fig = plt.figure(figsize=(15, 15),dpi=100); plt.suptitle('Positive Peaks: Windowed Magnitude Gyroscope',fontweight="bold");
    gs = GridSpec(nrows=3, ncols=3)
    ax0 = fig.add_subplot(gs[0, 0]); ax1 = fig.add_subplot(gs[0, 1]); ax2 = fig.add_subplot(gs[0, 2]);
    ax3 = fig.add_subplot(gs[1, 0]); ax4 = fig.add_subplot(gs[1, 1]); ax5 = fig.add_subplot(gs[1, 2]);
    ax6 = fig.add_subplot(gs[2, 0]); ax7 = fig.add_subplot(gs[2, 1]); ax8 = fig.add_subplot(gs[2, 2]);
    ax0.plot(sp_100_df['gyro']); ax0.plot(PP_sp_100_df, sp_100_df['gyro'].loc[PP_sp_100_df].to_numpy(),"x");
    ax1.plot(sp_300_df['gyro']); ax1.plot(PP_sp_300_df, sp_300_df['gyro'].loc[PP_sp_300_df].to_numpy(),"x");
    ax2.plot(sp_500_df['gyro']); ax2.plot(PP_sp_500_df, sp_500_df['gyro'].loc[PP_sp_500_df].to_numpy(),"x");
    ax3.plot(sp_700_df['gyro']); ax3.plot(PP_sp_700_df, sp_700_df['gyro'].loc[PP_sp_700_df].to_numpy(),"x");
    ax4.plot(sp_900_df['gyro']); ax4.plot(PP_sp_900_df, sp_900_df['gyro'].loc[PP_sp_900_df].to_numpy(),"x");
    ax5.plot(sp_1100_df['gyro']); ax5.plot(PP_sp_1100_df, sp_1100_df['gyro'].loc[PP_sp_1100_df].to_numpy(),"x");
    ax6.plot(sp_1300_df['gyro']); ax6.plot(PP_sp_1300_df, sp_1300_df['gyro'].loc[PP_sp_1300_df].to_numpy(),"x");
    ax7.plot(sp_1500_df['gyro']); ax7.plot(PP_sp_1500_df, sp_1500_df['gyro'].loc[PP_sp_1500_df].to_numpy(),"x");
    ax8.plot(sp_1700_df['gyro']); ax8.plot(PP_sp_1700_df, sp_1700_df['gyro'].loc[PP_sp_1700_df].to_numpy(),"x");
    ax0.set_title('Walking 60 step/min', fontweight="bold");
    ax1.set_title('Walking 90 step/min', fontweight="bold");
    ax2.set_title('Jogging 120 step/min', fontweight="bold");
    ax3.set_title('Running 150 step/min', fontweight="bold");
    ax4.set_title('Sprinting 180 step/min', fontweight="bold");
    ax5.set_title('Walking backwards 60 step/min', fontweight="bold");
    ax6.set_title('Walking backwards 90 step/min', fontweight="bold");
    ax7.set_title('Slideway walking right 90 step/min', fontweight="bold");
    ax8.set_title('Slideway walking left 90 step/min', fontweight="bold");

def plot_NP(sp_100_df,sp_300_df,sp_500_df,sp_700_df,sp_900_df,sp_1100_df,sp_1300_df,sp_1500_df,sp_1700_df,
            NP_sp_100_df,NP_sp_300_df,NP_sp_500_df,NP_sp_700_df,NP_sp_900_df,NP_sp_1100_df,NP_sp_1300_df,NP_sp_1500_df,NP_sp_1700_df):
    fig = plt.figure(figsize=(15, 15),dpi=100); plt.suptitle('Negative Peaks Algorithm: Windowed Magnitude Gyroscope',fontweight="bold");
    gs = GridSpec(nrows=3, ncols=3)
    ax0 = fig.add_subplot(gs[0, 0]); ax0.set_xlim([0, max(sp_100_df['timestamp'])]);
    ax1 = fig.add_subplot(gs[0, 1]); ax1.set_xlim([0, max(sp_300_df['timestamp'])]);
    ax2 = fig.add_subplot(gs[0, 2]); ax2.set_xlim([0, max(sp_500_df['timestamp'])]);
    ax3 = fig.add_subplot(gs[1, 0]); ax3.set_xlim([0, max(sp_700_df['timestamp'])]);
    ax4 = fig.add_subplot(gs[1, 1]); ax4.set_xlim([0, max(sp_900_df['timestamp'])]);
    ax5 = fig.add_subplot(gs[1, 2]); ax5.set_xlim([0, max(sp_1100_df['timestamp'])]);
    ax6 = fig.add_subplot(gs[2, 0]); ax6.set_xlim([0, max(sp_1300_df['timestamp'])]);
    ax7 = fig.add_subplot(gs[2, 1]); ax7.set_xlim([0, max(sp_1500_df['timestamp'])]);
    ax8 = fig.add_subplot(gs[2, 2]); ax8.set_xlim([0, max(sp_1700_df['timestamp'])]);
    ax0.plot(sp_100_df['timestamp'],sp_100_df['gyro']);
    ax0.vlines(sp_100_df['timestamp'].loc[NP_sp_100_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax1.plot(sp_300_df['timestamp'],sp_300_df['gyro']);
    ax1.vlines(sp_300_df['timestamp'].loc[NP_sp_300_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax2.plot(sp_500_df['timestamp'],sp_500_df['gyro']); 
    ax2.vlines(sp_500_df['timestamp'].loc[NP_sp_500_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax3.plot(sp_700_df['timestamp'],sp_700_df['gyro']); 
    ax3.vlines(sp_700_df['timestamp'].loc[NP_sp_700_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax4.plot(sp_900_df['timestamp'],sp_900_df['gyro']); 
    ax4.vlines(sp_900_df['timestamp'].loc[NP_sp_900_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax5.plot(sp_1100_df['timestamp'],sp_1100_df['gyro']);
    ax5.vlines(sp_1100_df['timestamp'].loc[NP_sp_1100_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax6.plot(sp_1300_df['timestamp'],sp_1300_df['gyro']);
    ax6.vlines(sp_1300_df['timestamp'].loc[NP_sp_1300_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax7.plot(sp_1500_df['timestamp'],sp_1500_df['gyro']); 
    ax7.vlines(sp_1500_df['timestamp'].loc[NP_sp_1500_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax8.plot(sp_1700_df['timestamp'],sp_1700_df['gyro']); 
    ax8.vlines(sp_1700_df['timestamp'].loc[NP_sp_1700_df], ymin = -1, ymax = 15,color='red',ls=':');
    ax0.set_title('Walking 60 step/min', fontweight="bold");
    ax1.set_title('Walking 90 step/min', fontweight="bold");
    ax2.set_title('Jogging 120 step/min', fontweight="bold");
    ax3.set_title('Running 150 step/min', fontweight="bold");
    ax4.set_title('Sprinting 180 step/min', fontweight="bold");
    ax5.set_title('Walking backwards 60 step/min', fontweight="bold");
    ax6.set_title('Walking backwards 90 step/min', fontweight="bold");
    ax7.set_title('Slideway walking right 90 step/min', fontweight="bold");
    ax8.set_title('Slideway walking left 90 step/min', fontweight="bold");

def plot_filter_max(sp_100_df,sp_300_df,sp_500_df,sp_700_df,sp_900_df,sp_1100_df,sp_1300_df,sp_1500_df,sp_1700_df,
                    df_peaks_100,df_peaks_300,df_peaks_500,df_peaks_700,df_peaks_900,df_peaks_1100,df_peaks_1300,df_peaks_1500,df_peaks_1700):
    fig = plt.figure(figsize=(15, 15),dpi=100); plt.suptitle('Filtered Maximums and Minimums',fontweight="bold");
    gs = GridSpec(nrows=3, ncols=3)
    ax0 = fig.add_subplot(gs[0, 0]); ax0.set_xlim([0, max(sp_100_df['timestamp'])]);
    ax1 = fig.add_subplot(gs[0, 1]); ax1.set_xlim([0, max(sp_300_df['timestamp'])]);
    ax2 = fig.add_subplot(gs[0, 2]); ax2.set_xlim([0, max(sp_500_df['timestamp'])]);
    ax3 = fig.add_subplot(gs[1, 0]); ax3.set_xlim([0, max(sp_700_df['timestamp'])]);
    ax4 = fig.add_subplot(gs[1, 1]); ax4.set_xlim([0, max(sp_900_df['timestamp'])]);
    ax5 = fig.add_subplot(gs[1, 2]); ax5.set_xlim([0, max(sp_1100_df['timestamp'])]);
    ax6 = fig.add_subplot(gs[2, 0]); ax6.set_xlim([0, max(sp_1300_df['timestamp'])]);
    ax7 = fig.add_subplot(gs[2, 1]); ax7.set_xlim([0, max(sp_1500_df['timestamp'])]);
    ax8 = fig.add_subplot(gs[2, 2]); ax8.set_xlim([0, max(sp_1700_df['timestamp'])]);
    ax0.plot(sp_100_df['timestamp'],sp_100_df['gyro']);
    ax0.vlines(df_peaks_100['timestamp'].loc[df_peaks_100['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax0.vlines(df_peaks_100['timestamp'].loc[df_peaks_100['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax1.plot(sp_300_df['timestamp'],sp_300_df['gyro']);
    ax1.vlines(df_peaks_300['timestamp'].loc[df_peaks_300['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax1.vlines(df_peaks_300['timestamp'].loc[df_peaks_300['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax2.plot(sp_500_df['timestamp'],sp_500_df['gyro']); 
    ax2.vlines(df_peaks_500['timestamp'].loc[df_peaks_500['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax2.vlines(df_peaks_500['timestamp'].loc[df_peaks_500['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax3.plot(sp_700_df['timestamp'],sp_700_df['gyro']); 
    ax3.vlines(df_peaks_700['timestamp'].loc[df_peaks_700['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax3.vlines(df_peaks_700['timestamp'].loc[df_peaks_700['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax4.plot(sp_900_df['timestamp'],sp_900_df['gyro']); 
    ax4.vlines(df_peaks_900['timestamp'].loc[df_peaks_900['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax4.vlines(df_peaks_900['timestamp'].loc[df_peaks_900['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax5.plot(sp_1100_df['timestamp'],sp_1100_df['gyro']);
    ax5.vlines(df_peaks_1100['timestamp'].loc[df_peaks_1100['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax5.vlines(df_peaks_1100['timestamp'].loc[df_peaks_1100['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax6.plot(sp_1300_df['timestamp'],sp_1300_df['gyro']);
    ax6.vlines(df_peaks_1300['timestamp'].loc[df_peaks_1300['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax6.vlines(df_peaks_1300['timestamp'].loc[df_peaks_1300['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax7.plot(sp_1500_df['timestamp'],sp_1500_df['gyro']); 
    ax7.vlines(df_peaks_1500['timestamp'].loc[df_peaks_1500['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax7.vlines(df_peaks_1500['timestamp'].loc[df_peaks_1500['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax8.plot(sp_1700_df['timestamp'],sp_1700_df['gyro']); 
    ax8.vlines(df_peaks_1700['timestamp'].loc[df_peaks_1700['label']=='PP'], ymin = -1, ymax = 16,color='teal',ls=':');
    ax8.vlines(df_peaks_1700['timestamp'].loc[df_peaks_1700['label']=='NP'], ymin = -1, ymax = 16,color='red',ls=':');
    ax0.set_title('Walking 60 step/min', fontweight="bold");
    ax1.set_title('Walking 90 step/min', fontweight="bold");
    ax2.set_title('Jogging 120 step/min', fontweight="bold");
    ax3.set_title('Running 150 step/min', fontweight="bold");
    ax4.set_title('Sprinting 180 step/min', fontweight="bold");
    ax5.set_title('Walking backwards 60 step/min', fontweight="bold");
    ax6.set_title('Walking backwards 90 step/min', fontweight="bold");
    ax7.set_title('Slideway walking right 90 step/min', fontweight="bold");
    ax8.set_title('Slideway walking left 90 step/min', fontweight="bold");

def plot_detection_zupt(sp_100_df,sp_300_df,sp_500_df,sp_700_df,sp_900_df,sp_1100_df,sp_1300_df,sp_1500_df,sp_1700_df,
                        x00,x10,x01,x11,x02,x12,x03,x13,x04,x14,x05,x15,x06,x16,x07,x17,x08,x18):
    fig = plt.figure(figsize=(15, 15),dpi=100); plt.suptitle('Detection ZUPT Zoom',fontweight="bold");
    gs = GridSpec(nrows=3, ncols=3)
    ax0 = fig.add_subplot(gs[0, 0]); ax0.set_xlim([x00, x10]);
    ax1 = fig.add_subplot(gs[0, 1]); ax1.set_xlim([x01, x11]);
    ax2 = fig.add_subplot(gs[0, 2]); ax2.set_xlim([x02, x12]);
    ax3 = fig.add_subplot(gs[1, 0]); ax3.set_xlim([x03, x13]);
    ax4 = fig.add_subplot(gs[1, 1]); ax4.set_xlim([x04, x14]);
    ax5 = fig.add_subplot(gs[1, 2]); ax5.set_xlim([x05, x15]);
    ax6 = fig.add_subplot(gs[2, 0]); ax6.set_xlim([x06, x16]);
    ax7 = fig.add_subplot(gs[2, 1]); ax7.set_xlim([x07, x17]);
    ax8 = fig.add_subplot(gs[2, 2]); ax8.set_xlim([x08, x18]);
    ax0.scatter(sp_100_df['timestamp'].loc[sp_100_df['label']=='walk_60'], sp_100_df['gyro'].loc[sp_100_df['label']=='walk_60'], c='b', s=.01)
    ax0.scatter(sp_100_df['timestamp'].loc[sp_100_df['label']=='ZUPT'], sp_100_df['gyro'].loc[sp_100_df['label']=='ZUPT'], c='g', s=.01)
    ax0.scatter(sp_100_df['timestamp'].loc[sp_100_df['label']=='ZUPT_walk_60'], sp_100_df['gyro'].loc[sp_100_df['label']=='ZUPT_walk_60'], c='r', s=.01)
    ax1.scatter(sp_300_df['timestamp'].loc[sp_300_df['label']=='walk_90'], sp_300_df['gyro'].loc[sp_300_df['label']=='walk_90'], c='b', s=.01)
    ax1.scatter(sp_300_df['timestamp'].loc[sp_300_df['label']=='ZUPT'], sp_300_df['gyro'].loc[sp_300_df['label']=='ZUPT'], c='g', s=.01)
    ax1.scatter(sp_300_df['timestamp'].loc[sp_300_df['label']=='ZUPT_walk_90'], sp_300_df['gyro'].loc[sp_300_df['label']=='ZUPT_walk_90'], c='r', s=.01)
    ax2.scatter(sp_500_df['timestamp'].loc[sp_500_df['label']=='jogg_120'], sp_500_df['gyro'].loc[sp_500_df['label']=='jogg_120'], c='b', s=.01)
    ax2.scatter(sp_500_df['timestamp'].loc[sp_500_df['label']=='ZUPT'], sp_500_df['gyro'].loc[sp_500_df['label']=='ZUPT'], c='g', s=.01)
    ax2.scatter(sp_500_df['timestamp'].loc[sp_500_df['label']=='ZUPT_jogg_120'], sp_500_df['gyro'].loc[sp_500_df['label']=='ZUPT_jogg_120'], c='r', s=.01)
    ax3.scatter(sp_700_df['timestamp'].loc[sp_700_df['label']=='run_150'], sp_700_df['gyro'].loc[sp_700_df['label']=='run_150'], c='b', s=.01)
    ax3.scatter(sp_700_df['timestamp'].loc[sp_700_df['label']=='ZUPT'], sp_700_df['gyro'].loc[sp_700_df['label']=='ZUPT'],  c='g', s=.01)
    ax3.scatter(sp_700_df['timestamp'].loc[sp_700_df['label']=='ZUPT_run_150'], sp_700_df['gyro'].loc[sp_700_df['label']=='ZUPT_run_150'],  c='r', s=.01)
    ax4.scatter(sp_900_df['timestamp'].loc[sp_900_df['label']=='sprint_180'], sp_900_df['gyro'].loc[sp_900_df['label']=='sprint_180'], c='b', s=.01)
    ax4.scatter(sp_900_df['timestamp'].loc[sp_900_df['label']=='ZUPT'], sp_900_df['gyro'].loc[sp_900_df['label']=='ZUPT'], c='g', s=.01)
    ax4.scatter(sp_900_df['timestamp'].loc[sp_900_df['label']=='ZUPT_sprint_180'], sp_900_df['gyro'].loc[sp_900_df['label']=='ZUPT_sprint_180'], c='r', s=.01)
    ax5.scatter(sp_1100_df['timestamp'].loc[sp_1100_df['label']=='walk_b_60'], sp_1100_df['gyro'].loc[sp_1100_df['label']=='walk_b_60'], c='b', s=.01)
    ax5.scatter(sp_1100_df['timestamp'].loc[sp_1100_df['label']=='ZUPT'], sp_1100_df['gyro'].loc[sp_1100_df['label']=='ZUPT'], c='g', s=.01)
    ax5.scatter(sp_1100_df['timestamp'].loc[sp_1100_df['label']=='ZUPT_walk_b_60'], sp_1100_df['gyro'].loc[sp_1100_df['label']=='ZUPT_walk_b_60'], c='r', s=.01)
    ax6.scatter(sp_1300_df['timestamp'].loc[sp_1300_df['label']=='walk_b_90'], sp_1300_df['gyro'].loc[sp_1300_df['label']=='walk_b_90'], c='b', s=.01)
    ax6.scatter(sp_1300_df['timestamp'].loc[sp_1300_df['label']=='ZUPT'], sp_1300_df['gyro'].loc[sp_1300_df['label']=='ZUPT'], c='g', s=.01)
    ax6.scatter(sp_1300_df['timestamp'].loc[sp_1300_df['label']=='ZUPT_walk_b_90'], sp_1300_df['gyro'].loc[sp_1300_df['label']=='ZUPT_walk_b_90'], c='r', s=.01)
    ax7.scatter(sp_1500_df['timestamp'].loc[sp_1500_df['label']=='slide_right'], sp_1500_df['gyro'].loc[sp_1500_df['label']=='slide_right'], c='b', s=.01)
    ax7.scatter(sp_1500_df['timestamp'].loc[sp_1500_df['label']=='ZUPT'], sp_1500_df['gyro'].loc[sp_1500_df['label']=='ZUPT'], c='g', s=.01)
    ax7.scatter(sp_1500_df['timestamp'].loc[sp_1500_df['label']=='ZUPT_slide_right'], sp_1500_df['gyro'].loc[sp_1500_df['label']=='ZUPT_slide_right'], c='r', s=.01)
    ax8.scatter(sp_1700_df['timestamp'].loc[sp_1700_df['label']=='slide_left'], sp_1700_df['gyro'].loc[sp_1700_df['label']=='slide_left'], c='b', s=.01)
    ax8.scatter(sp_1700_df['timestamp'].loc[sp_1700_df['label']=='ZUPT'], sp_1700_df['gyro'].loc[sp_1700_df['label']=='ZUPT'],  c='g', s=.01)
    ax8.scatter(sp_1700_df['timestamp'].loc[sp_1700_df['label']=='ZUPT_slide_left'], sp_1700_df['gyro'].loc[sp_1700_df['label']=='ZUPT_slide_left'],  c='r', s=.01)
    ax0.set_title('Walking 60 step/min', fontweight="bold");
    ax1.set_title('Walking 90 step/min', fontweight="bold");
    ax2.set_title('Jogging 120 step/min', fontweight="bold");
    ax3.set_title('Running 150 step/min', fontweight="bold");
    ax4.set_title('Sprinting 180 step/min', fontweight="bold");
    ax5.set_title('Walking backwards 60 step/min', fontweight="bold");
    ax6.set_title('Walking backwards 90 step/min', fontweight="bold");
    ax7.set_title('Slideway walking right 90 step/min', fontweight="bold");
    ax8.set_title('Slideway walking left 90 step/min', fontweight="bold");