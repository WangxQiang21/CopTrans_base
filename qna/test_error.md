个人测试中遇到的问题：
1.p11_case_a：
末端航向不对，我指的方向是水平向右，这个停的好像不是
![alt text](image-8.png)
这里可以直行到中间，然后spin掉头，然后o模式斜移动到库中，或者是斜行到库中，轨迹长度也应该加入cost
![alt text](image-9.png)
这里先a模式拐个大弯，然后o模式就很奇怪：
![alt text](image-10.png)

普遍问题：末端的航向不对，有模型奇妙的a模式大转弯

2.p11_case_b
这里都没有解：
![alt text](image-11.png)
这里有解：![alt text](image-12.png)但是![alt text](image-13.png)航向反过来就没有解，是不是因为禁止倒车引起的，我们不应该把倒车禁止掉

3.p11_case_c
![alt text](image-14.png)，这里直接倒车过去就行，我们不能禁止倒车，倒车更方便的时候是可以使用的

4.p12_env1
![alt text](image-15.png)里面的![alt text](image-16.png)明明可以o模式过来的，为什么非要用a模式拐弯，这里已经有spin-a的转换了，时间损失已经存在了，用o模式路径更短,尤其是短的两个a-spin-a,如果中间有a模式的大曲率，o模式路径更短，直接过去，就用o模式，如果说不用模式切换,a模式直接可以走，那还是用a模式

4.p12_env2
![alt text](image-17.png)无解，而且![alt text](image-18.png)地图有锯齿

5.p12_env3
![alt text](image-19.png)倒车不应该禁止 
![alt text](image-20.png)末端精度太差：![alt text](image-21.png)

5.p12_env4
![alt text](image-22.png)末端精度差：![alt text](image-23.png)