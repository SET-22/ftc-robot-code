import React from 'react';
import clsx from 'clsx';

interface GamepadSliderProps {
  label: string;
  value: number;
  keyBinding?: string;
  onChange: (value: number) => void;
  className?: string;
}

export const GamepadSlider: React.FC<GamepadSliderProps> = ({
  label,
  value,
  keyBinding,
  onChange,
  className,
}) => {
  const handleSliderChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    onChange(parseFloat(e.target.value));
  };

  const handleDoubleClick = () => {
    // Reset to 0 on double click
    onChange(0);
  };

  return (
    <div className={clsx('flex flex-col items-center gap-1', className)}>
      <div className="flex w-full items-center gap-1.5">
        <span className="min-w-[24px] text-xs font-medium text-gray-700 dark:text-gray-300">
          {label}
        </span>
        <input
          type="range"
          min="0"
          max="1"
          step="0.01"
          value={value}
          onChange={handleSliderChange}
          onDoubleClick={handleDoubleClick}
          className={clsx(
            'h-2 flex-1 cursor-pointer appearance-none rounded-lg',
            'bg-gray-200 dark:bg-gray-700',
            '[&::-webkit-slider-thumb]:appearance-none',
            '[&::-webkit-slider-thumb]:w-4',
            '[&::-webkit-slider-thumb]:h-4',
            '[&::-webkit-slider-thumb]:rounded-full',
            '[&::-webkit-slider-thumb]:bg-blue-500',
            '[&::-webkit-slider-thumb]:cursor-pointer',
            '[&::-webkit-slider-thumb]:transition-all',
            '[&::-webkit-slider-thumb]:hover:bg-blue-600',
            '[&::-webkit-slider-thumb]:active:bg-blue-700',
            '[&::-moz-range-thumb]:w-4',
            '[&::-moz-range-thumb]:h-4',
            '[&::-moz-range-thumb]:rounded-full',
            '[&::-moz-range-thumb]:bg-blue-500',
            '[&::-moz-range-thumb]:border-0',
            '[&::-moz-range-thumb]:cursor-pointer',
            '[&::-moz-range-thumb]:transition-all',
            '[&::-moz-range-thumb]:hover:bg-blue-600',
            '[&::-moz-range-thumb]:active:bg-blue-700',
          )}
          title={
            keyBinding
              ? `Key: ${keyBinding} | Double-click to reset`
              : 'Double-click to reset'
          }
        />
        <span className="min-w-[36px] text-right text-xs font-medium text-gray-600 dark:text-gray-400">
          {(value * 100).toFixed(0)}%
        </span>
      </div>
      {keyBinding && (
        <span className="ml-[28px] self-start text-[9px] text-gray-500 dark:text-gray-400">
          {keyBinding}
        </span>
      )}
    </div>
  );
};
