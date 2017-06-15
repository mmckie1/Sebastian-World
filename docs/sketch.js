var bgImg02;
var bgImg01;

var player0;
var player;

var collectibles;
var koopa00;
var enemy;

var audio;
var jumpSound;
var coinSound;
var wonSound;

var goal01;
var goal02;
var goal_slider;
var finish;

var enemyDead;

function preload() {
  
  audio = loadSound("assets/audio/Overworld.mp3");
  jumpSound = loadSound("assets/audio/Jump.mp3");
  dieSound = loadSound("assets/audio/Dead mario.mp3");
  wonSound = loadSound("assets/audio/Course-clear.mp3") 
  coinSound = loadSound("assets/audio/Coin.mp3");
  
  //load background image
  bgImg02 = loadImage("assets/sky_2.png");
  bgImg01 = loadImage("assets/sky.png");
}

function setup() {
  createCanvas(500,600);
  
  enemyDead = false;
  
  jumpSound.setVolume(0.5);
  dieSound.setVolume(0.5);
  coinSound.setVolume(0.5);
  audio.setVolume(0.5);
  audio.play();
  
  //load ground
  groundImg = createSprite(width/2, 390);
  groundImg.addImage(loadImage("assets/top_ground.png"));
  
  goal01 = createSprite(1020,286);
  goal01.addImage(loadImage("assets/Goal-front.gif"))
  
  goal02 = createSprite(985,286);
  goal02.addImage(loadImage("assets/Goal-back.gif"));
  
  goal_slider = createSprite(1000,345);
  goal_slider.addImage(loadImage("assets/Goal-slider.gif"));
  
  collectibles = new Group();
  for (var i=0; i<10; i++){
    var coins = createSprite(random(0,1000),345);
    coins.addAnimation("idle","assets/coins/coins_01.png","assets/coins/coins_02.png","assets/coins/coins_03.png","assets/coins/coins_04.png");
    collectibles.add(coins);
  }
  
  //create player sprite
  player = createSprite(0,250);

  //load different states of player
  player.addAnimation("standing", "assets/Sebastion_idle.png");
  player.addAnimation("running", "assets/Sebastion_running_01.png", "assets/Sebastion_running_02.png", "assets/Sebastion_running_03.png");
  player.addAnimation("jumping", "assets/Sebastion_jumping.png");
  player.addAnimation("dead", "assets/Sebastion_dead01.png","assets/Sebastion_dead01.png");
  
  //create player object 
  player0 = new Sebastian(player);
  
  enemy = createSprite(300,345);
  enemy.addAnimation("walking", "assets/enemy/koopa_standing.png","assets/enemy/koppa_walking.png","assets/enemy/koopa_standing.png");
  enemy.addAnimation("squish", "assets/enemy/squish_koppa02.png","assets/enemy/squish_koopa01.png","assets/enemy/squish_koppa02.png");
  
  koopa00 = new Koopa(enemy);
  
  finish = new FinishLine(goal_slider);
  
  var mgr = new SceneManager();
  mgr.bgImg02 = bgImg02;
  mgr.bgImg01 = bgImg01;
  mgr.wire();
  mgr.showScene(Intro);
}

