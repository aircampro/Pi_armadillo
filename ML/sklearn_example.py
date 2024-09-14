#!/usr/bin/env python
#
# example of ML using sklearn
#
# foo.csv may be like below :-
# 1,1.2,12
# 1,2.0,5
# 1,1.5,3
# 2,1.8,1
# 2,0.5,10
# 2,1.0,8
#
import argparse
import numpy as np
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
import sys

# parse command line
parser = argparse.ArgumentParser()
parser.add_argument('--ml_model', '-m', type=int, default=0, help='ML model 0=svm,1=NN,2=LR,3=DTC,4=GBC,5=KNC,6=NB,7=SGDC, 8=TPOT, 9-11=RFC, 12=RFR')
parser.add_argument('--test_train_type', '-tts', type=int, default=0, help='test train split 0=features,1=xy')

# read and pre-process data
data = np.loadtxt('foo.csv', delimiter=',', dtype=float)
labels = data[:, 0:1]                                                                           # first column from csv
labelsY = data[:, 1:2]                                                                          # second column from csv
features = preprocessing.minmax_scale(data[:, 1:])                                              # 説明変数を取り出した上でスケーリング this is 2nd 2 colums normalised [[0.46666667, 1.],[1., 0.36363636],etc[0.33333333, 0.63636364]])

# choosethe test train split
if test_train_type == 0:
    x_train, x_test, y_train, y_test = train_test_split(features, labels.ravel(), test_size=0.3) 
else:
    x_train, x_test, y_train, y_test = train_test_split(labels, LabelsY, test_size=0.2, random_state=42)

# choose model for classification or regression
if ml_model == 0 :
    from sklearn import svm
    clf = svm.SVC(kernel='rbf', C=10, gamma=0.1) 
elif ml_model == 1 :
    from sklearn import neural_network
    clf = neural_network.MLPClassifier(activation="relu", alpha=0.0001) 
elif ml_model == 2 :
    from sklearn.linear_model import LogisticRegression
    clf = LogisticRegression(random_state=42)
elif ml_model == 3 :
    from sklearn.tree import DecisionTreeClassifier
    clf = DecisionTreeClassifier()
elif ml_model == 4 :
    from sklearn.ensemble import GradientBoostingClassifier
    clf = GradientBoostingClassifier()
elif ml_model == 5 :
    from sklearn.neighbors import KNeighborsClassifier
    clf = KNeighborsClassifier()
elif ml_model == 6 :
    from sklearn.naive_bayes import GaussianNB
    clf = GaussianNB()
elif ml_model == 7 :
    from sklearn.linear_model import SGDClassifier
    clf = SGDClassifier()
elif ml_model == 8 :    
    from tpot import TPOTClassifier
    clf = TPOTClassifier(scoring='f1', generations=3, population_size=50, verbosity=2, n_jobs=-1)
elif ml_model == 9 :    
    from sklearn.ensemble import RandomForestClassifier  
    clf = RandomForestClassifier(random_state=1234)
elif ml_model == 10 :    
    from sklearn.ensemble import RandomForestClassifier  
    clf = RandomForestClassifier(n_estimators=100, random_state=0)
elif ml_model == 11 :    
    from sklearn.ensemble import RandomForestClassifier  
    clf = RandomForestClassifier(bootstrap=True,ccp_alpha=0.0,class_weight=None,criterion=’gini’,max_depth=None,max_ features=’auto’,max_leaf_nodes=None,max_samples=None,min_impurity_decrease=0.0, min_impurity_split=None,min_ samples_leaf=1,min_samples_split=2,min_weight_fraction_leaf=0.0,n_estimators=5,n_jobs=None,oob_score=False,random_state=2,verbose=0, warm_start=False)
elif ml_model == 12 : 
    from sklearn.ensemble import RandomForestRegressor
    clf = RandomForestRegressor(n_estimators=100,criterion='mse',max_depth=None,min_samples_split=2,min_samples_leaf=1,min_weight_fraction_leaf=0.0,max_features='auto',max_leaf_nodes=None,min_impurity_decrease=0.0,bootstrap=True,oob_score=False,n_jobs=None,random_state=None,verbose=0,warm_start=False,ccp_alpha=0.0,max_samples=None)
else:
    print("number of models is 0-12")
    sys.exit(-1)
	
# for chosen model fit the training data
#
clf.fit(x_train, y_train)

if ml_model == 8 :
    # Confirmation of the best pipeline finally adopted
    clf.fitted_pipeline_
    # Checking the pipeline during learning
    clf.evaluated_individuals_

# prediction and accuracy using the test set of data
#
from sklearn.metrics import accuracy_score, precision_score, recall_score

predict = clf.predict(x_test)
print(accuracy_score(y_test, predict), precision_score(y_test, predict), recall_score(y_test, predict))

# make 2 classifications on data set input
# the input is columns 2 and 3 from the example file
# 
fetures_test_data = [[1.8636667, 13],[3.32, 10.267]]
predict = clf.predict(preprocessing.minmax_scale(fetures_test_data))
print(predict)
