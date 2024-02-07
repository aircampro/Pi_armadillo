# Example of prediction using svm support vector machine, tpot, random forrest, decision tree
# weather_learn.py
#
import numpy as np
from sklearn import svm

# csv file containg the test/train data
# in order from left to right Temperature, precipitation, sunshine hours, humidity, 
# From left to right, temperature, precipitation, sunshine hours, humidity, and weather (0: sunny, 1: cloudy, 2: rainy)
# weather_data.csv   ---- probably realistically need to collect a lot more data
# 6.6,0,8.2,47,0
# 7.1,0,5.7,57,1
# 7.1,0,9.3,62,0
# 8.1,0,4.7,53,1
# 6.5,0,9.7,54,0
# 8,0,8.1,42,1
# 6.6,1.5,0.5,68,2
# 5.7,21.5,2.7,94,2
# 11.2,0,9.3,47,0
# 9,0,7.9,57,1
# 8,0,4.5,66,1
# 7.7,0,1.8,66,2
# 9.1,0,9.3,70,0
# 9.1,0,8.3,70,0
# 7.8,11.5,3.6,79,2
# 7.6,0,4.4,46,1
# 7.6,0,3.6,58,1
# 3.8,13.5,0,87,2
# 7.3,0,8,62,1
# 8.3,0,9.7,60,0
#
npArray = np.loadtxt("weather_data.csv", delimiter = ",", dtype = "float")

# select the criteria we are measuring as the x-azis
x = npArray[:, 0:4]

# select the result as the y axis
y = npArray[:, 4:5].ravel()

# create the svm model
model = svm.SVC()

# fit all the x and y data
model.fit(x,y)

# this is what we want to predict its the current weather data
weather = [[9,0,7.9,6.5]] 

# predict the weather using this weather data measuremnt given above.
ans = model.predict(weather)

print("results using support vector machine SVM model")
if ans == 0:
    print("sunny")
elif ans == 1:
    print("cloudy")
elif ans == 2:
    print("rainy")

# Tpot with random forrest
#
from tpot import TPOTClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import f1_score
from sklearn.metrics import confusion_matrix

# split the data into a test and train set for model validation
X_train, X_test, y_train, y_test = train_test_split(x, y, train_size=0.8, test_size=0.2, random_state=42)

# Define TPOT classifier and parameters
# scoring: Indicator used for Cross Validation
# generation: number of generations
# population_size : Number of pipelines generated in each generation
# verbosity: Displays the status during learning (4 levels from 0 to 3, the higher the number, the more detailed)
# n_jobs: Parallelism of learning processing (use all cores by specifying -1)

model_tpot = TPOTClassifier(scoring='f1',
                            generations=3,
                            population_size=50,
                            verbosity=2,
                            n_jobs=-1)

# fit the tpot model
model_tpot.fit(X_train, y_train)

# Confirmation of the best pipeline finally adopted
model_tpot.fitted_pipeline_

# Checking the pipeline during learning
model_tpot.evaluated_individuals_

# Evaluate the built model using evaluation data (metric is f1 score)
y_pred = model_tpot.predict(X_test)
f1_score(y_true=y_test, y_pred=y_pred)

# confusion matrix
confusion_matrix(y_pred=y_pred,y_true=y_test)

# now usae the current weather measuments to predict a discrete classification of weather
ans = model_tpot.predict(weather)

print("results using tpot with random forrest model")
if ans == 0:
    print("sunny")
elif ans == 1:
    print("cloudy")
elif ans == 2:
    print("rainy")

# random forrest classifier models
from sklearn.ensemble import RandomForestClassifier  

from sklearn.metrics import r2_score            # (R^2)
from sklearn.metrics import mean_squared_error  # RMSE

X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.3, random_state=1234)

# create the random forrest classification model you can test various settings
#
clf = RandomForestClassifier(random_state=1234)
#clf = RandomForestClassifier(n_estimators=100, random_state=0)         # 100 estimators 
#clf = RandomForestClassifier(bootstrap=True,
#                             ccp_alpha=0.0,
#                             class_weight=None,
#                             criterion=’gini’,
#                             max_depth=None,
#                             max_ features=’auto’,
#                             max_leaf_nodes=None,
#                             max_samples=None,
#                             min_impurity_decrease=0.0,
#                             min_impurity_split=None,
#                             min_ samples_leaf=1,
#                             min_samples_split=2,
#                             min_weight_fraction_leaf=0.0,
#                             n_estimators=5,
#                             n_jobs=None,
#                             oob_score=False,
#                             random_state=2,
#                             verbose=0,
#                             warm_start=False)

clf.fit(X_train, y_train)

print("Accuracy on training set: {:.3f}".format(clf.score(X_train, y_train)))
print("Accuracy on test set: {:.3f}".format(clf.score(X_test, y_test)))

# Calculate RMSE for model
# (Train）
y_train_pred = clf.predict(X_train)

# （Test)
y_test_pred = clf.predict(X_test)

# (RMSE)
print('RMSE train: %.2f, test: %.2f' % (
        mean_squared_error(y_train, y_train_pred, squared=False), # train
        mean_squared_error(y_test, y_test_pred, squared=False)    # test
      ))

# (R^2)
print('R^2 train: %.2f, test: %.2f' % (
        r2_score(y_train, y_train_pred), # train
        r2_score(y_test, y_test_pred)    # test
      ))

# now estimate our weather     
ans = clf.predict(weather)

print("results using random forrest classifier model")
if ans == 0:
    print("sunny")
elif ans == 1:
    print("cloudy")
elif ans == 2:
    print("rainy")

# random forrest regression
from sklearn.ensemble import RandomForestRegressor

X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.4, random_state=1)

forest = RandomForestRegressor(n_estimators=100,
                               criterion='mse', 
                               max_depth=None, 
                               min_samples_split=2, 
                               min_samples_leaf=1, 
                               min_weight_fraction_leaf=0.0, 
                               max_features='auto', 
                               max_leaf_nodes=None, 
                               min_impurity_decrease=0.0, 
                               bootstrap=True, 
                               oob_score=False, 
                               n_jobs=None, 
                               random_state=None, 
                               verbose=0, 
                               warm_start=False, 
                               ccp_alpha=0.0, 
                               max_samples=None
                              )
# use training data to fit the random forrest regresor model
forest.fit(X_train, y_train)  

# Calculate RMSE for model
# (Train）
y_train_pred = forest.predict(X_train)

# （Test)
y_test_pred = forest.predict(X_test)

# (RMSE)
print('RMSE train: %.2f, test: %.2f' % (
        mean_squared_error(y_train, y_train_pred, squared=False), # train
        mean_squared_error(y_test, y_test_pred, squared=False)    # test
      ))

# (R^2)
print('R^2 train: %.2f, test: %.2f' % (
        r2_score(y_train, y_train_pred), # train
        r2_score(y_test, y_test_pred)    # test
      ))

f1_score(y_true=y_test, y_pred=y_test_pred) 
      
ans = forest.predict(weather)
print("answer : ", ans)                 

print("results using random forrest regression model")
if ans >= 0 and ans <= 0.5 :
    print("sunny")
elif ans >= 0.5 and ans <= 1.5:
    print("cloudy")
elif ans >= 1.5 and ans <= 2:
    print("rainy")

# Tpot with Decision Tree Classifier
#
from sklearn.tree import DecisionTreeClassifier

X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.4, random_state=1)

model = DecisionTreeClassifier( criterion='gini',
                                splitter='best',
                                max_depth=3,
                                min_samples_split=3,
                                min_samples_leaf=1,
                                min_weight_fraction_leaf=0.0,
                                max_features=4,
                                random_state=None,
                                max_leaf_nodes=8,
                                min_impurity_split=1e-07,
                                class_weight='balanced',
                                presort=False)
model.fit(X_train, y_train)

# Calculate RMSE for model
# (Train）
y_train_pred = model.predict(X_train)

# （Test)
y_test_pred = model.predict(X_test)

# root mean square error (RMSE)
print('RMSE train: %.2f, test: %.2f' % (
        mean_squared_error(y_train, y_train_pred, squared=False), # train
        mean_squared_error(y_test, y_test_pred, squared=False)    # test
      ))

# (R^2)
print('R^2 train: %.2f, test: %.2f' % (
        r2_score(y_train, y_train_pred), # train
        r2_score(y_test, y_test_pred)    # test
      ))
      
f1_score(y_true=y_test, y_pred=y_test_pred)  

ans = model.predict(weather)

print("results using decision tree classifier model")
if ans == 0:
    print("sunny")
elif ans == 1:
    print("cloudy")
elif ans == 2:
    print("rainy")