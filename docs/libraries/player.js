var gravity = 3;
var jump = -5;
var GROUND_Y = 350;
var SCENE_W = 1024;
var SCENE_H = 400;
var score = 0;

function Sebastian(tempSprite) {
  this.sprite = tempSprite;
  
  this.create = function(curState){
     //gravity
    this.sprite.velocity.y = gravity;
    
    if(keyIsDown(LEFT_ARROW) || keyIsDown(RIGHT_ARROW)){
      if (keyIsDown(LEFT_ARROW)){
        //face and move left 
        this.sprite.mirrorX(-1);
        this.sprite.velocity.x = -2;
      } else if (keyIsDown(RIGHT_ARROW)){
        //face and move right 
        this.sprite.mirrorX(1);
        this.sprite.velocity.x = 2;
      }
      this.sprite.changeAnimation("running");
    } else {
      this.sprite.velocity.x = 0;
    }
    
    //if player is on the ground and not moving left or right 
    //the animation is standing else he is running
    if (curState === true){
      if (this.sprite.velocity.x === 0){
        this.sprite.changeAnimation("standing");
      } else {
        this.sprite.changeAnimation("running");
      }
    }
    
    //jump controls 
    if (keyWentDown(UP_ARROW) && curState === true){
        this.sprite.velocity.y = -50;
        this.sprite.changeAnimation("jumping");
        jumpSound.play();
      } 
    
     if(this.sprite.position.x < 0){    
      this.sprite.position.x = 0;  
     }
     if(this.sprite.position.y < 0){    
      this.sprite.position.y = 0;  
     }
     if(this.sprite.position.x > 1000){
      this.sprite.position.x = 1000;
      Winner(this.sprite);
     }
     if(this.sprite.position.y > SCENE_H){
      this.sprite.position.y = SCENE_H;
     }
     
    camera.position.x = this.sprite.position.x;
    camera.position.y = 200;
    camera.zoom = 1.5;
    
    finish.controls();
    
    this.sprite.overlap(collectibles,collect);
    
    Score(this.sprite.position.x);
    
    koopa00.controls();
    this.sprite.collide(enemy, StepOnEnemy) 
    
  }
   this.check = function(){
      
    if(this.sprite.collide(groundImg)){
      return true;
    } else {
      return false;
    }
    
  }
}

//function sets the boundaries for koopa Enemy 
function Koopa(tempEne) {
  this.ene = tempEne;
  
  this.controls = function(){  
  
  
    if (this.ene.position.x <= 300){
      this.ene.position.x = 300;
      this.ene.mirrorX(-1);
      this.ene.velocity.x = +1;
    }
    if (this.ene.position.x >= 450){
      this.ene.position.x = 450;
      this.ene.mirrorX(1);
      this.ene.velocity.x = -1;
    }

  }
  
  
}

//function for winnig the game 
function Winner(tempThing){
  audio.stop();
  wonSound.play();
  tempThing.remove();
  //gameOver();
}

function FinishLine(tempfin) {
  this.fin = tempfin;
  
  this.controls = function(){
  
    if (this.fin.position.y <= 250) {
      this.fin.position.y = 250;
      this.fin.velocity.y = 1;
    }
    if (this.fin.position.y >= 345) {
      this.fin.position.y = 345;
      this.fin.velocity.y = -1;
    }
  
    
  }
  
}

 function Score(tempX){
  fill("#ffffff").strokeWeight(2).textSize(10);
  stroke(0);
  textFont("Georgia");
  text("Coins: " + score ,tempX-150,19);
}

//remove tokens when ovelapped and plays token sound
function collect(collector,collected) {
  collected.remove();
  coinSound.play();
  score++;
}

function StepOnEnemy(player, enemy){
  
  var player_Left = leftSide(player);
  var player_Right = rightSide(player);
  var player_Up = upSide(player);
  var player_Down = downSide(player);
  
  var enemy_Left = leftSide(enemy);
  var enemy_Right = rightSide(enemy);
  var enemy_Up = upSide(enemy);
  var enemy_Down = downSide(enemy);
  
  if(player_Down<=enemy_Up) {
    enemyDead = true;
  }
  if(enemyDead){
    enemy.remove();
  }

  if(player_Right>=enemy_Right){
    playerDead = true;
  } 
  if(playerDead){
    player.remove();
  }

  console.log("Sebastian's Right: " + player_Right);
  console.log("Koppa's Right: " + enemy_Right);
}


function leftSide(obj) {
  return obj.position.x-(obj.width/2);
}

function rightSide(obj) {
  return obj.position.x+(obj.width/2);
}

function upSide(obj) {
  return obj.position.y-(obj.width/2);
}

function downSide(obj) {
  return obj.position.y+(obj.width/2);
}