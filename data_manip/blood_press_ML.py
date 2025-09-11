# ML Model to predict actual blood pressure from 'alpha','PIR', 'ptt'
#
import pandas as pd 
from sklearn import datasets
from sklearn.model_selection import train_test_split 
from sklearn.linear_model import LinearRegression  
import numpy as np  
from sklearn.metrics import mean_squared_error, r2_score
import csv
from argparse import ArgumentParser

CSV_DATA = "medical_data_file.csv"
ML_MODEL = "RIDGE"                                   # choose ML model to use for data regression

# load csv file with experiment recording (e.g. for replay)
def load_csv_recording(file_path):
    print('Loading file {}'.format(file_path))
    try:
        data: pd.DataFrame = pd.read_csv(file_path, comment='#', names = ['alpha','PIR', 'ptt', 'bpmax' ,'bpmin', 'hrfinal', 'ih', 'il', 'meu', 'j', 'k','l','m','n','o','p','q','r','s'])  # skip comment lines starting with #
    except Exception as e:
        print('Cannot load: Caught {} trying to read CSV file {}'.format(e, file_path))
        return False
    # Change to float32 wherever numeric column
    cols = data.columns
    data[cols] = data[cols].apply(pd.to_numeric, errors='ignore', downcast='float')
    return data

parser = ArgumentParser()
parser.add_argument("--csvfile", metavar="<csv file>")
parser.add_argument("--ml_model", metavar="<machine learning model for regression>")
if args.csvfile:
    CSV_DATA = args.csvfile 
if args.ml_model:
    ML_MODEL = args.ml_model     
# read the data from a csv logfile
dataset = load_csv_recording(CSV_DATA)
X = dataset[['alpha','PIR', 'ptt']]
y = dataset[['bpmin','bpmax']]
sbp = list()
dbp = list()
# calculate real blood pressure from max and min
real_BP = list()
with open(CSV_DATA, 'r') as csvfile:
	csv_reader = csv.reader(csvfile, delimiter = ',')
	print(csv_reader)
	for row in csv_reader:
		sbp.append(float(row[3]))
		dbp.append(float(row[4]))
	real_BP = list()
	for i in range(len(sbp)):
		BP_actual = (2*dbp[i] + sbp[i])/3
		real_BP.append(BP_actual)

# split data into train and test sets
X_train, X_test, y_train, y_test = train_test_split(X, real_BP, test_size=0.2, random_state=0) 
# choose the machine learning model and predict using the test set data
if ML_MODEL == "LR":
    from sklearn.preprocessing import StandardScaler
    sc_X=StandardScaler()
    x_train=sc_X.fit_transform(X_train)
    x_test=sc_X.transform(X_test)
    regressor = LinearRegression() 
    regressor.fit(x_train, y_train)
    y_pred = regressor.predict(x_test)
elif ML_MODEL == "SVR":
    from sklearn.preprocessing import StandardScaler
    sc_X=StandardScaler()
    X=sc_X.fit_transform(X)
    from sklearn.svm import SVR
    regressor = SVR(kernel = 'rbf')
    regressor.fit(X_train, y_train)
    y_pred = regressor.predict(X_test)
elif ML_MODEL == "XGB":
    from sklearn.preprocessing import StandardScaler
    sc_X=StandardScaler()
    x_train=sc_X.fit_transform(X_train)
    x_test=sc_X.transform(X_test)
    from xgboost import XGBClassifier
    regressor = XGBClassifier()
    regressor.fit(x_train, y_train)
    y_pred = regressor.predict(x_test)
elif ML_MODEL == "DTR":
    from sklearn.preprocessing import StandardScaler
    sc_X=StandardScaler()
    x_train=sc_X.fit_transform(X_train)
    x_test=sc_X.transform(X_test)
    from sklearn.tree import DecisionTreeRegressor
    from sklearn.ensemble import AdaBoostRegressor
    rng = np.random.RandomState(1)
    regr_1 = DecisionTreeRegressor(max_depth=4)
    regr_2 = AdaBoostRegressor(DecisionTreeRegressor(max_depth=4), n_estimators=300, random_state=rng)
    regr_1.fit(X_train, y_train)
    regr_2.fit(X_train, y_train)
    # Predict
    y_pre = regr_1.predict(X_test)
    y_pred = regr_2.predict(X_test)
elif ML_MODEL == "RIDGE":
    from sklearn.linear_model import Ridge
    from sklearn.preprocessing import StandardScaler
    sc_X=StandardScaler()
    x_train=sc_X.fit_transform(X_train)
    x_test=sc_X.transform(X_test)
    regressor = Ridge(normalize=True)
    regressor.fit(X_train, y_train)
    y_pred = regressor.predict(X_test)
elif ML_MODEL == "NN":
    from sklearn.preprocessing import StandardScaler
    from sklearn.neural_network import MLPRegressor
    sc_X=StandardScaler()
    x_train=sc_X.fit_transform(X_train)
    x_test=sc_X.transform(X_test)
    for i in range(5,100):
	    for j in range(5,100):
		    nn = MLPRegressor(
	        hidden_layer_sizes=(i,j,),  activation='relu', solver='adam', alpha=0.001, batch_size='auto',
	        learning_rate='constant', learning_rate_init=0.01, power_t=0.5, max_iter=1000, shuffle=True,
	        random_state=9, tol=0.0001, verbose=False, warm_start=False, momentum=0.9, nesterovs_momentum=True,
	        early_stopping=False, validation_fraction=0.1, beta_1=0.9, beta_2=0.999, epsilon=1e-08)
		    nn.fit(x_train, y_train)
		    y_pred = nn.predict(x_test)
elif ML_MODEL = "RFC":
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.preprocessing import StandardScaler
    sc_X=StandardScaler()
    x_train=sc_X.fit_transform(X_train)
    x_test=sc_X.transform(X_test)
    regressor = RandomForestRegressor(n_estimators=100,criterion='mse',max_depth=None,min_samples_split=2,min_samples_leaf=1,min_weight_fraction_leaf=0.0,max_features='auto',max_leaf_nodes=None,min_impurity_decrease=0.0,bootstrap=True,oob_score=False,n_jobs=None,random_state=None,verbose=0,warm_start=False,ccp_alpha=0.0,max_samples=None) 
    regressor.fit(x_train, y_train)
    y_pred = regressor.predict(x_test)
else:
    print("invalid model must be one of:- NN RIDGE DTR XGB SVR LR RFC")
# print metrics
from sklearn import metrics  
print('Mean Absolute Error:', metrics.mean_absolute_error(y_test, y_pred))  
print('Mean Squared Error:', metrics.mean_squared_error(y_test, y_pred))  
print('Root Mean Squared Error:', np.sqrt(metrics.mean_squared_error(y_test, y_pred)))

# # Explained variance score: 1 is perfect prediction
print('Variance score: %.2f' % r2_score(y_test, y_pred))