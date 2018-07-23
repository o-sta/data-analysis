# AR.Drone 2.0
AR.Drone 2.0 のデータを解析する

## 手順

1.  データ取得
```bash
rosbag record /ardrone/navdata /transition
```

2.  `/transition`トピックをcsvに変換
```bash
rostopic echo -b bagファイル.bag -p /transition > csvファイル.csv
```

3.  MATLABで`navdata_read.m`の変数等を編集し、実行する
