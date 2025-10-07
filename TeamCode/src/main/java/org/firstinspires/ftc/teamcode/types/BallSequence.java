package org.firstinspires.ftc.teamcode.types;

public enum BallSequence {
  GPP(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE),
  PGP(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE),
  PPG(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN);

  private final BallColor[] ballColors;

  BallSequence(BallColor... ballColors) {
    this.ballColors = ballColors;
  }

  public BallColor[] getBallColors() {
    return ballColors.clone();
  }
}
