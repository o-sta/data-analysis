# data-analysis
ROSのbagファイルをmatlabで読み込み、データ解析やグラフの出力を行うためのリポジトリ

手順

1.  データ取得
''' rosbag record /ardrone/navdata /transition '''

2.  '/transition'トピックをcsvに変換
''' rostopic echo -b bagファイル.bag -p /transition > csvファイル.csv '''

3.  MATLABで'navdata_read.m'の変数等を編集し、実行する

4.  結果が保存される
