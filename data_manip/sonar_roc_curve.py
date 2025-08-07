# example of ROC Curve for analysing sonar data to try to determine rocks or mines in the sea
# ref :- https://lightningchart.com/blog/python/sonar-object-detection/
#
import pandas as pd
import lightningchart as lc
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import train_test_split
from sklearn.metrics import roc_curve, auc
import sys

if len(sys.argv[0]) > 0:
    filen = str(sys.argv[1])
else:
    filen = 'sonar.csv'
data = pd.read_csv(filen, header=None)
X = data.drop(columns=[60])
y = data[60].apply(lambda x: 1 if x == 'M' else 0)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
model = LogisticRegression()
model.fit(X_train, y_train)
y_scores = model.predict_proba(X_test)[:, 1]
fpr, tpr, _ = roc_curve(y_test, y_scores)
roc_auc = auc(fpr, tpr)
chart = lc.ChartXY(title='Logistic Regression ROC Curve')
series = chart.add_line_series().add(fpr.tolist(), tpr.tolist()).set_name('ROC Curve')
chart.open()