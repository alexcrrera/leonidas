



void calculateOffsets(int offsetTool){
  switch(offsetTool){
    case 1: // vectornav angles
      angleOffsetX = vectornavAngleX;
      angleOffsetY = vectornavAngleY;
      angleOffsetZ = vectornavAngleZ;
    break;

    case 2: // pressure reset
    
    break;

    case 3: // reset lidar 
      positionZOffset = lidarReadings[1];
      break;

    case 4:
      positionXOffset = positionX;
      positionYOffset = positionY;
      positionZOffset = lidarReadings[1];//positionZ; as long as we're not using rtk.-..


    default:
      return;


  }

}