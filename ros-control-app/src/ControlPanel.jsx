import React from 'react';

const ControlPanel = ({ handleButtonClick }) => {
  const handleClick = (direction) => {
    handleButtonClick(direction);
  };

  return (
    <div>
      <button onClick={() => handleClick('forward')}>Forward</button>
      <button onClick={() => handleClick('backward')}>Backward</button>
      <button onClick={() => handleClick('left')}>Left</button>
      <button onClick={() => handleClick('right')}>Right</button>
    </div>
  );
};

export default ControlPanel;
