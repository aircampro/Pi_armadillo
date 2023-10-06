// ref to site :: http://cvwww.ee.ous.ac.jp/processing_prog/#s23

// calib correction ref:- http://pukulab.blog.fc2.com/blog-entry-22.html

PImage f, g;                                         // 画像オブジェクトf,gを宣言する f=in g=out
int[] histR = new int[256];                          // 整数型配列histRを宣言する（赤ヒストグラム） 
int[] histG = new int[256];                          // 整数型配列histGを宣言する（緑ヒストグラム）
int[] histB = new int[256];                          // 整数型配列histBを宣言する（青ヒストグラム）
 
void setup() {
  int   x, y, i;                                     // 整数型変数x,y,iを宣言する
  int   minR, minG, minB, maxR, maxG, maxB;          // 整数型変数minR,minG,minB,maxR,maxG,maxBを宣言する
  color c;                                           // color型変数cを宣言する
  float a, s;                                        // 実数型変数a,sを宣言する
  float R, G, B;                                     // 実数型変数R,G,Bを宣言する
  float[] tableR = new float[256];                   // 実数型配列tableRを宣言する（赤のルックアップテーブル用）
  float[] tableG = new float[256];                   // 実数型配列tableGを宣言する（緑のルックアップテーブル用）
  float[] tableB = new float[256];                   // 実数型配列tableBを宣言する（青のルックアップテーブル用）
  
  f = loadImage("image.jpg");                        // 原画像を画像fに読み込む
  g = createImage(f.width, f.height, RGB);           // 画像gを画像fと同じサイズで作る
  surface.setSize(f.width*2, f.height);              // 実行画面のサイズを原画像の2倍の幅に設定する
 
  // ヒストグラムを計算する
  for (y = 0; y < f.height; y++) {                   // yを0から画像縦幅-1まで1ずつ増やす
    for (x = 0; x < f.width; x++) {                  // xを0から画像横幅-1まで1ずつ増やす
      c = f.get(x, y);                               // 画像fの座標(x,y)の色の値（color型）をcに代入する
      histR[int(red(c))  ]++;                        // cから赤画素値を取り出し、histRに個数をカウントする
      histG[int(green(c))]++;                        // cから緑画素値を取り出し、histGに個数をカウントする
      histB[int(blue(c)) ]++;                        // cから青画素値を取り出し、histBに個数をカウントする
    }
  }
 
  // 累積ヒストグラムを計算する
  for (i = 1; i < 256; i++) {                        // iを1から255まで1ずつ増やす
    histR[i] += histR[i-1];                          // histR[i]にhistR[i-1]の値を加算する
    histG[i] += histG[i-1];                          // histG[i]にhistG[i-1]の値を加算する
    histB[i] += histB[i-1];                          // histB[i]にhistB[i-1]の値を加算する
  }
 
  // ヒストグラムから画素値の最小値と最大値を得る。ただし、ヒストグラムの右端と左端から割合aを除いた範囲とする
  a = .02;                                           // aに計算から除くヒストグラムの範囲の割合を[0,.5)の範囲で指定する
  s = histR[255];                                    // sにhistR[255]の値（画像の全画素数）を代入する
  minR = minG = minB = 0;                            // 画素値の最小値minR,minG,minBを0で初期化する
  maxR = maxG = maxB = 255;                          // 画素値の最大値maxR,maxG,maxBを255で初期化する
  while (histR[minR]/s < a && minR < 255) minR++;    // 赤の最小画素値を求めてminRに入れる
  while (histG[minG]/s < a && minG < 255) minG++;    // 緑の最小画素値を求めてminGに入れる
  while (histB[minB]/s < a && minB < 255) minB++;    // 青の最小画素値を求めてminBに入れる
  a = 1 - a;                                         // aに1-aの値を代入する
  while (histR[maxR]/s > a && maxR > 0) maxR--;      // 赤の最大画素値を求めてmaxRに入れる
  while (histG[maxG]/s > a && maxG > 0) maxG--;      // 緑の最大画素値を求めてmaxGに入れる
  while (histB[maxB]/s > a && maxB > 0) maxB--;      // 青の最大画素値を求めてmaxBに入れる
  
  // ヒストグラム伸長を行うルックアップテーブルをR,G,Bごとに作成する
  for (i = 0; i < 256; i++) {                        // iを0から255まで1ずつ増やす
    tableR[i] = 255./(maxR-minR)*(i-minR);           // 赤画素値のヒストグラム伸長の変換値を求めてtableRに代入する
    tableG[i] = 255./(maxG-minG)*(i-minG);           // 緑画素値のヒストグラム伸長の変換値を求めてtableGに代入する
    tableB[i] = 255./(maxB-minB)*(i-minB);           // 青画素値のヒストグラム伸長の変換値を求めてtableBに代入する
  }
 
  // R,G,Bごとにヒストグラム伸長を行う
  for (y = 0; y < f.height; y++) {                   // yを0から画像縦幅-1まで1ずつ増やす
    for (x = 0; x < f.width; x++) {                  // xを0から画像横幅-1まで1ずつ増やす
      c = f.get(x, y);                               // 画像fの座標(x,y)の色の値（color型）をcに代入する
      R = tableR[int(red(c))];                       // 赤の画素値に対応する変換値をtableRから取り出してRに代入する
      G = tableG[int(green(c))];                     // 緑の画素値に対応する変換値をtableGから取り出してRに代入する
      B = tableB[int(blue(c))];                      // 青の画素値に対応する変換値をtableBから取り出してRに代入する
      g.set(x, y, color(R, G, B));                   // R,G,Bから作ったcolor型の値を画像gの座標(x,y)の画素に設定する
    }
  }
 
  g.save("output.jpg");                              // 画像gをファイルoutput.jpgに保存する
}
 
void draw() {
  image(f, 0, 0);                                    // 実行画面の左側に原画像fを貼る
  image(g, f.width, 0);                              // 実行画面の右側に変換後画像gを貼る
}
Python mode
def setup():
  global f, g                                        # 変数f,gのグローバル宣言
  f = loadImage("image.jpg")                         # 原画像を画像fに読み込む
  g = createImage(f.width, f.height, RGB)            # 画像gを画像fと同じサイズで作る
  this.surface.setSize(f.width*2, f.height)          # 実行画面のサイズを原画像の2倍の幅に設定する

  # ヒストグラムを計算する
  histR = [0]*256                                    # リストhistRを宣言する（赤ヒストグラム）
  histG = [0]*256                                    # リストhistGを宣言する（緑ヒストグラム）
  histB = [0]*256                                    # リストhistBを宣言する（青ヒストグラム）
  for y in range(0, f.height):                       # yを0から画像縦幅-1まで1ずつ増やす
    for x in range(0, f.width):                      # xを0から画像横幅-1まで1ずつ増やす
      c = f.get(x, y)                                # 画像fの座標(x,y)の色の値（color型）をcに代入する
      histR[int(red(c))  ] += 1                      # cから赤画素値を取り出し、histRに個数をカウントする
      histG[int(green(c))] += 1                      # cから緑画素値を取り出し、histGに個数をカウントする
      histB[int(blue(c)) ] += 1                      # cから青画素値を取り出し、histBに個数をカウントする

  # 累積ヒストグラムを計算する
  for i in range(1, 256): # iを1から255まで1ずつ増やす
    histR[i] += histR[i-1]                           # histR[i]にhistR[i-1]の値を加算する
    histG[i] += histG[i-1]                           # histG[i]にhistG[i-1]の値を加算する
    histB[i] += histB[i-1]                           # histB[i]にhistB[i-1]の値を加算する

  # ヒストグラムから画素値の最小値と最大値を得る。ただし、ヒストグラムの右端と左端から割合aを除いた範囲とする
  a = .02                                            # aに計算から除くヒストグラムの範囲の割合を[0,.5)の範囲で指定する
  s = float(histR[255])                              # sにhistR[255]の値（画像の全画素数）を代入する
  minR = minG = minB = 0                             # 画素値の最小値minR,minG,minBを0で初期化する
  maxR = maxG = maxB = 255                           # 画素値の最大値maxR,maxG,maxBを255で初期化する
  while histR[minR]/s < a and minR < 255: minR += 1  # 赤の最小画素値を求めてminRに入れる
  while histG[minG]/s < a and minG < 255: minG += 1  # 緑の最小画素値を求めてminGに入れる
  while histB[minB]/s < a and minB < 255: minB += 1  # 青の最小画素値を求めてminBに入れる
  a = 1 - a                                          # aに1-aの値を代入する
  while histR[maxR]/s > a and maxR > 0: maxR -= 1    # 赤の最大画素値を求めてmaxRに入れる
  while histG[maxG]/s > a and maxG > 0: maxG -= 1    # 緑の最大画素値を求めてmaxGに入れる
  while histB[maxB]/s > a and maxB > 0: maxB -= 1    # 青の最大画素値を求めてmaxBに入れる

  # ヒストグラム伸長を行うルックアップテーブルをR,G,Bごとに作成する
  tableR = [255./(maxR-minR)*(i-minR) for i in range(0, 256)] # 赤画素値のヒストグラム伸長の変換値を求めてtableRに代入する
  tableG = [255./(maxG-minG)*(i-minG) for i in range(0, 256)] # 緑画素値のヒストグラム伸長の変換値を求めてtableGに代入する
  tableB = [255./(maxB-minB)*(i-minB) for i in range(0, 256)] # 青画素値のヒストグラム伸長の変換値を求めてtableBに代入する

  # R,G,Bごとにヒストグラム伸長を行う
  for y in range(0, f.height):                       # yを0から画像縦幅-1まで1ずつ増やす
    for x in range(0, f.width):                      # xを0から画像横幅-1まで1ずつ増やす
      c = f.get(x, y)                                # 画像fの座標(x,y)の色の値（color型）をcに代入する
      R = tableR[int(red(c))]                        # 赤の画素値に対応する変換値をtableRから取り出してRに代入する
      G = tableG[int(green(c))]                      # 緑の画素値に対応する変換値をtableGから取り出してRに代入する
      B = tableB[int(blue(c))]                       # 青の画素値に対応する変換値をtableBから取り出してRに代入する
      g.set(x, y, color(R, G, B))                    # R,G,Bから作ったcolor型の値を画像gの座標(x,y)の画素に設定する
    
  g.save("output.jpg")                               # 画像gをファイルoutput.jpgに保存する
  
def draw():
  image(f, 0, 0)                                     # 実行画面の左側に原画像fを貼る
  image(g, f.width, 0)                               # 実行画面の右側に変換後画像gを貼る
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
def setup():
  global f, g                                        # 変数f,gのグローバル宣言
  f = loadImage("image.jpg")                         # 原画像を画像fに読み込む
  g = createImage(f.width, f.height, RGB)            # 画像gを画像fと同じサイズで作る
  this.surface.setSize(f.width*2, f.height)          # 実行画面のサイズを原画像の2倍の幅に設定する
 
  # ヒストグラムを計算する
  histR = [0]*256                                    # リストhistRを宣言する（赤ヒストグラム）
  histG = [0]*256                                    # リストhistGを宣言する（緑ヒストグラム）
  histB = [0]*256                                    # リストhistBを宣言する（青ヒストグラム）
  for y in range(0, f.height):                       # yを0から画像縦幅-1まで1ずつ増やす
    for x in range(0, f.width):                      # xを0から画像横幅-1まで1ずつ増やす
      c = f.get(x, y)                                # 画像fの座標(x,y)の色の値（color型）をcに代入する
      histR[int(red(c))  ] += 1                      # cから赤画素値を取り出し、histRに個数をカウントする
      histG[int(green(c))] += 1                      # cから緑画素値を取り出し、histGに個数をカウントする
      histB[int(blue(c)) ] += 1                      # cから青画素値を取り出し、histBに個数をカウントする
 
  # 累積ヒストグラムを計算する
  for i in range(1, 256): # iを1から255まで1ずつ増やす
    histR[i] += histR[i-1]                           # histR[i]にhistR[i-1]の値を加算する
    histG[i] += histG[i-1]                           # histG[i]にhistG[i-1]の値を加算する
    histB[i] += histB[i-1]                           # histB[i]にhistB[i-1]の値を加算する
 
  # ヒストグラムから画素値の最小値と最大値を得る。ただし、ヒストグラムの右端と左端から割合aを除いた範囲とする
  a = .02                                            # aに計算から除くヒストグラムの範囲の割合を[0,.5)の範囲で指定する
  s = float(histR[255])                              # sにhistR[255]の値（画像の全画素数）を代入する
  minR = minG = minB = 0                             # 画素値の最小値minR,minG,minBを0で初期化する
  maxR = maxG = maxB = 255                           # 画素値の最大値maxR,maxG,maxBを255で初期化する
  while histR[minR]/s < a and minR < 255: minR += 1  # 赤の最小画素値を求めてminRに入れる
  while histG[minG]/s < a and minG < 255: minG += 1  # 緑の最小画素値を求めてminGに入れる
  while histB[minB]/s < a and minB < 255: minB += 1  # 青の最小画素値を求めてminBに入れる
  a = 1 - a                                          # aに1-aの値を代入する
  while histR[maxR]/s > a and maxR > 0: maxR -= 1    # 赤の最大画素値を求めてmaxRに入れる
  while histG[maxG]/s > a and maxG > 0: maxG -= 1    # 緑の最大画素値を求めてmaxGに入れる
  while histB[maxB]/s > a and maxB > 0: maxB -= 1    # 青の最大画素値を求めてmaxBに入れる
 
  # ヒストグラム伸長を行うルックアップテーブルをR,G,Bごとに作成する
  tableR = [255./(maxR-minR)*(i-minR) for i in range(0, 256)] # 赤画素値のヒストグラム伸長の変換値を求めてtableRに代入する
  tableG = [255./(maxG-minG)*(i-minG) for i in range(0, 256)] # 緑画素値のヒストグラム伸長の変換値を求めてtableGに代入する
  tableB = [255./(maxB-minB)*(i-minB) for i in range(0, 256)] # 青画素値のヒストグラム伸長の変換値を求めてtableBに代入する
 
  # R,G,Bごとにヒストグラム伸長を行う
  for y in range(0, f.height):                       # yを0から画像縦幅-1まで1ずつ増やす
    for x in range(0, f.width):                      # xを0から画像横幅-1まで1ずつ増やす
      c = f.get(x, y)                                # 画像fの座標(x,y)の色の値（color型）をcに代入する
      R = tableR[int(red(c))]                        # 赤の画素値に対応する変換値をtableRから取り出してRに代入する
      G = tableG[int(green(c))]                      # 緑の画素値に対応する変換値をtableGから取り出してRに代入する
      B = tableB[int(blue(c))]                       # 青の画素値に対応する変換値をtableBから取り出してRに代入する
      g.set(x, y, color(R, G, B))                    # R,G,Bから作ったcolor型の値を画像gの座標(x,y)の画素に設定する
    
  g.save("output.jpg")                               # 画像gをファイルoutput.jpgに保存する
  
def draw():
  image(f, 0, 0)                                     # 実行画面の左側に原画像fを貼る
  image(g, f.width, 0)                               # 実行画面の右側に変換後画像gを貼る