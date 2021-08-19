# connect_product_strategy_mobile


## ROS Server <terminal 1> 
```
cd ~/merchandise_stock_disposal
. devel/setup.bash
chmod +x src/connect_product_strategy_mobile/src/ProductStrategy_Mobile_server.py

rosrun connect_product_strategy_mobile ProductStrategy_Mobile_server.py
```

## ROS Client <terminal 2> 
```
cd ~/merchandise_stock_disposal
. devel/setup.bash
chmod +x src/connect_product_strategy_mobile/src/ProductStrategy_Mobile_client.py

(1) Test1: if forget to input place
rosrun connect_product_strategy_mobile ProductStrategy_Mobile_server.py

(2) Test2: input place not assigned (i.e.: Bathroom)
rosrun connect_product_strategy_mobile ProductStrategy_Mobile_server.py Bathroom

(3) Test3: input place assigned (i.e.: Home, Table1, Table2, Shelf)
rosrun connect_product_strategy_mobile ProductStrategy_Mobile_server.py Home

```