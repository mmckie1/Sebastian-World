function Intro() {
  
  this.setup = function() {
    
  }
  this.draw = function() {
      //load blue background color for the sky
      background(color(0,100,190));
      //load background image 02 and 01. Order in which they are called is important.  
      image(this.sceneManager.bgImg02,-450,70);
      image(this.sceneManager.bgImg01,-450,70);
      
      //check if player is colliding with ground
      var curPlayerState = player0.check();
      //load player with controlls 
      player0.create(curPlayerState);
      
      //draw different game sprites
      drawSprites();
      
      
  }
}

