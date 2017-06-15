function Intro() {
  
  this.setup = function() {
    
  }
  this.draw = function() {
      background(color(0,100,190));
      image(this.sceneManager.bgImg02,-450,70);
      image(this.sceneManager.bgImg01,-450,70);
      
      //check if player is colliding with ground
      var curPlayerState = player0.check();
      //load player with controlls 
      player0.create(curPlayerState);
      
      
      drawSprites();
      
      
  }
}

