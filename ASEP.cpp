#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define TEXT_MODE // 標準出力にテキストでセルを描画
#define IMAGE_MODE // 画像でセルを描画
#define SIMULATION // 密度、速度、流量を標準出力に出す

using namespace std;

/**
 * @brief 乱数発生用クラス
 */
class Random{
public:
  Random(){
    srand( static_cast<unsigned int>( time(NULL) ) );
  }
  
  unsigned int operator()(unsigned int max){
    double tmp = static_cast<double>( rand() ) / static_cast<double>( RAND_MAX );
    return static_cast<unsigned int>( tmp * max );
  }
};


/**
 * @brief ASEP(ルール184)モデルにスロースタートとクイックスタートを追加
 */
class CellularAutomaton{

public:

  CellularAutomaton(int map_size, int car_num):
    map_size_(map_size), car_num_(car_num)
  {}
  ~CellularAutomaton(){}

  /**
   * @brief 車の種類。数字は動き出しの遅さを表す。
   */
  enum CarType{
    NONE   = 0,
    SMALL  = 1,
    MIDDLE = 2,
    LARGE  = 5
  };
  
  /**
   * @brief セルの状態
   */
  struct State{
    bool	car;		// 車がいるかどうか
    bool	move;		// 車が動けるかどうか
    CarType	type;		// どの種類の車か
    int		count;		// 0になったら発進できるカウンタ
  };

  /**
   * @brief セルオートマトンを実行する関数
   */
  void do_simulation(int num){

    init_state();
    int time_tick = 500;
    
    // Loop
    for(int t=0; t<num; t++){

      // add noise ====
      // stop_car(); // 確率的に車をとめる
      // slow_car(); // 確率的に車を遅くする

      // check state ====
      // check_state(); // 前のセルが空なら動く <-- クイック無し
      check_state_quick(); // 前のセルの車が動くなら動く <-- クイック有り

      // output ====
#ifdef TEXT_MODE
      dump_move();
      dump_car();
      dump_count();
#endif

#ifdef IMAGE_MODE
      draw_image();
#endif

#ifdef CALC_MODE
      if( t%time_tick == 0 && t != 0 ){
	cout << calc_density() << "\t" << calc_average_speed() << "\t" << q_var_/time_tick << endl;
	q_var_ = 0.0; // 流量の初期化
      }
#endif
      
      // update ====
      step_state_rev(); // 車を動かす
      reset_move();     // moveをリセットする
    }
  }
   
private:

  // === member ===
  int		map_size_;	// 地図のセル数
  int		car_num_;	// 車の台数
  double	q_var_;		// ind = 0での流量
  cv::Mat	img_;		// 道路の描画
  vector<State> state_map_;	// 地図データ
  
  
  /**
   * @brief 地図をゼロにする関数
   */
  void zero_state(vector<State>& map){
    State state = {false, false, NONE};
    int size = map.size();
    for(int i=0; i<size; i++){
      map[i] = state;
    }
  }
  
  /**
   * @brief 地図を初期化する関数
   */
  void init_state(){ // vector<State>& state_map){
    
    state_map_.resize(map_size_);
    zero_state(state_map_);
    q_var_ = 0;
    
    // 前からcar numだけ車を詰める
    for(int i=0; i<map_size_; i++){
      if( i < car_num_ ){
	state_map_[i].car = true;
	state_map_[i].move = false;
      }else{
	state_map_[i].car = false;
	state_map_[i].move = false;
      }

      // 車の種類を設定
      std::random_device rnd; // 非決定的な乱数生成器
      std::mt19937 mt(rnd()); // メルセンヌツイスター
      std::uniform_int_distribution<> rand100(0, 99); // [0, 99] 範囲の一様乱数
      int rnd_num = rand100(mt);
      
      if( rnd_num%3 == 0 ){
	state_map_[i].type = SMALL;
      }
      if( rnd_num%3 == 1 ){
	state_map_[i].type = MIDDLE;
      }
      if( rnd_num%3 == 2 ){
	state_map_[i].type = LARGE;
      }

      // 車がないところはNONEにする
      if( state_map_[i].car == false ){
	state_map_[i].type = NONE;
      }
      state_map_[i].count = state_map_[i].type;
      
    }

    // 順番をランダムにする
    Random r;
    random_shuffle(state_map_.begin(), state_map_.end(), r);
  }

  
  /**
   * @brief
   */
  void reset_move(){ 
    int size = state_map_.size();
    for(int i=0; i<size; i++){
      state_map_[i].move = false;
    }
  }
  
  
  /**
   * @brief
   */
  void dump_car(){

    int size = state_map_.size();
    cout << "[";
    for(int i=0; i<size; i++){
      if( state_map_[i].car == true ){

	if( state_map_[i].type == SMALL )
	  cout << "S";
	else if( state_map_[i].type == MIDDLE )
	  cout << "M";
	else if( state_map_[i].type == LARGE )
	  cout << "L";
	// cout << "*";
      }else{
      	cout << " ";
      }
    }
    cout << "]";
    cout << endl;
  }
  
  /**
   * @brief
   */  
  void dump_move(){
    int size = state_map_.size();
    cout << "[";
    for(int i=0; i<size; i++){
      if( state_map_[i].move == true ){
	cout << ".";
      }else{
	cout << " ";
      }
    }
    cout << "]";
    cout << endl;
  }

  /**
   * @brief
   */  
  void dump_count(){
    int size = state_map_.size();
    cout << "[";
    for(int i=0; i<size; i++){
      cout << state_map_[i].count;
    }
    cout << "]";
    cout << endl;
  }

  
  /**
   * @brief i番目のセルのn個先のセルに車があるかどうかを確認する関数
   */
  bool check_forward_car(int i, int n){ 
    int size = state_map_.size();
    int ind = (i+n)%size;
    return state_map_[ind].car;
  }

  /**
   * @brief i番目のセルのn個先のセルが動ける状態かどうかを確認する関数
   */
  bool check_forward_move(int i, int n){
    int size = state_map_.size();
    int ind = (i+n)%size;
    return state_map_[ind].move;
  }
  
  
  /**
   * @brief あるセルが動ける状態かどうかを確認する関数
   *        動ける状態だった場合はmove = trueとする
   */
  void check_state(){
    
    int size = state_map_.size();
    
    for(int i=0; i<size; i++){
      
      if( check_forward_car(i , 0) == true ){ // 自分のセルに車がある 
	if( check_forward_car(i , 1) == false ){ // 前のセルに車がない
	  if( state_map_[i].count <= 0 ){ // カウントが0かどうか
	    state_map_[i].move = true;
	  }else{
	    state_map_[i].count--;
	  }
	}else{
	  state_map_[i].count = state_map_[i].type; // 初期化
	}
      }
    }
  }

  
  /**
   * @brief クイックスタートモデル
   *        N個先のセルがmoveなら自身もmoveとする
   */ 
  void check_state_quick(){

    int size = state_map_.size();
    for(int i=0; i<size; i++){
      
      if( check_forward_car(i , 0) == true ){ // 自分のセルに車がある
	if( check_forward_car(i , 1) == false ){ // 前のセルに車がない
	  if( state_map_[i].count <= 0 ){ // カウントが0かどうか
	    state_map_[i].move = true;
	  }else{
	    state_map_[i].count--;
	  }
	}else{  // クイックスタート

	  if( check_forward_move(i , 1) == true ){ // 前のセルがmove
	    if( state_map_[i].count == 0 ){ // カウントが0
	      state_map_[i].move = true;
	    }else{ // カウントが0でない
	      state_map_[i].count--;
	    }
	  }else{ // 前のセルがmoveじゃない（詰まっている）
	    state_map_[i].count = state_map_[i].type; // 前が詰まっているときは初期化
	  }
	}
      }
    }
  }
  
  /**
   * @brief 逆順にたどりながら各セルの状態をチェックして状態を変更する
   */
  void step_state_rev(){

    int size = state_map_.size();
    State tail_state1 = state_map_[size-1];
    State tail_state2 = state_map_[size-2];
    State tail_state3 = state_map_[size-3];
    
    for(int i=size-1; i>=0; --i){

      int ind = (size + i)%(size); 
      int p_ind = (size + i-1)%(size); // 一つ前のindex

      if( state_map_[p_ind].move == true ){

	state_map_[ind].car   = true;
	state_map_[ind].type = state_map_[p_ind].type;

	state_map_[p_ind].car = false;
	state_map_[p_ind].move = false;
	state_map_[p_ind].type = NONE;

	// ループの接続部分の処理
	if( ind == 0 ){
	  state_map_[ind] = tail_state1;

	  // Qの計算用
	  q_var_+=1.0;

	  // TODO:見通す数によって変化させねばならない
	  if( tail_state2.move == true ){
	    state_map_[p_ind].car  = true;
	    state_map_[p_ind].type  = tail_state2.type;
	  }else
	    state_map_[p_ind].car  = false;
	  
	}
      }else{
      }
    }
  }

  
  /**
   * @brief 画像で表示
   */
  void draw_image(){

    // 描画用変数
    int		w_scale	   = 8;
    int		h_scale	   = 16;
    int		car_scale  = 6;
    int		margin	   = 4;
    int		img_width  = state_map_.size()*w_scale + 2;
    int		img_height = 1*h_scale + margin*2;
    double	car_ratio  = 0.5;
    int		car_width  = 1*w_scale-margin;
    int		car_height = car_width*car_ratio;
    int		bin_width  = 1*w_scale;
    int		bin_height = 1*h_scale;
    
    img_ = cv::Mat::zeros(cv::Size(img_width, img_height),CV_8UC3);

    for(int ind = 0; ind<state_map_.size(); ++ind){

      cv::Point lt_pnt = {ind*bin_width+1,      img_height/2};
      cv::Point rb_pnt = {lt_pnt.x + car_width, lt_pnt.y + car_height};
      cv::Point lb_pnt = {ind*bin_width+1,      lt_pnt.y + car_height};
      cv::Point rt_pnt = {lt_pnt.x + car_width, img_height/2};
	
      cv::Scalar car_color;
      cv::Scalar state_color;      
      
      if( state_map_[ind].car == true ){

	// 種類ごとに大きさを変える
	if( state_map_[ind].type == SMALL  ){
	  car_color = {0,150,180};
	  lt_pnt = lt_pnt + cv::Point(1,1);
	  rb_pnt = rb_pnt - cv::Point(1,1);
	}else if( state_map_[ind].type == LARGE ){
	  car_color = {0,200,0};
	  lt_pnt = lt_pnt - cv::Point(1,1);
	  rb_pnt = rb_pnt + cv::Point(1,1);
	}else if( state_map_[ind].type == MIDDLE ){
	  car_color = {200,0,0};
	}

	// 車の描画
	cv::rectangle(img_, lt_pnt, rb_pnt, car_color, -1, 4);

	// 停止中に赤色の枠を付ける
	if( state_map_[ind].move == false ){
	  state_color = {0, 0, 200};
	  cv::rectangle(img_, lt_pnt, rb_pnt, state_color,  1, 0);
	}
	
      }else{ // 車じゃないセル
	; // do nothing
      }
    }
      
    cv::imshow("img", img_);
    cv::waitKey(100);
  }

  
  /**
   * @brief 確率的に車が停車してカウントが初期化される
   *        どこかの車のcountが確率的に増える
   */
  void stop_car(){

    std::random_device rnd; // 非決定的な乱数生成器
    std::mt19937 mt(rnd()); // メルセンヌツイスター
    std::uniform_int_distribution<> rand(0,199); // [0, 199] 範囲の一様乱数

    int size = state_map_.size();
    for( int i=0; i<size; i++){
      if( state_map_[i].car == true ){
	if( rand(mt) == 100 ){
	  state_map_[i].count = state_map_[i].type;
	}
      }
    }
  }
  
  /**
   * @brief 確率的に車が減速してカウントが少し増える
   */
  void slow_car(){//vector<State>& state_map_){

    std::random_device rnd; // 非決定的な乱数生成器
    std::mt19937 mt(rnd()); // メルセンヌツイスター
    std::uniform_int_distribution<> rand(0,199); // [0, 199] 範囲の一様乱数

    // どれかの車
    int size = state_map_.size();
    for( int i=0; i<size; i++){
      if( state_map_[i].car == true ){
	if( rand(mt) == 10 ){
	  state_map_[i].count += 1;
	}
      }
    }
  }

  /**
   * @brief 前方Nセルにブレーキランプを見たらcountを1足す
   */
  void detect_brake_lamp(){//vector<State>& state_map_){

    // 自身がmove==trueの状態で
    
    // 前方Nセルのmoveをチェックして

    // もしmove==falseがあれば、自身のカウントを1増やす
  }

  /**
   * @brief 平均速度を計算する関数
   */
  double calc_average_speed(){
    int size = state_map_.size();
    double ave_speed = 0.0;
    for(int i=0; i<size; ++i){
      if( state_map_[i].move == true )
	ave_speed += 1.0;
    }
    return ave_speed / (double)car_num_;
  }

  /**
   * @brief 密度を計算する関数
   */
  double calc_density(){
    return (double)car_num_ / (double)map_size_;
  }
  
};


int main(int argc, char* argv[]){

  // int map_size  = atoi(argv[1]);
  // int car_num   = atoi(argv[2]);
  // int simu_time = atoi(argv[3]);

  // for ( int car=0; car<car_num; car += 1){
  //   CellularAutomaton ca(
  // 			 map_size,
  // 			 car
  // 			 );
  //   ca.do_simulation(simu_time); 
  // }


  int map_size  = 100;
  int car_num   = 30;
  int simu_time = 1000;
  
  CellularAutomaton ca(map_size, car_num);
  ca.do_simulation(simu_time); 
  
  
  return 0;
}
