library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RoboticArmFourDof is
  port (
    -- Clock: 50MHz από εσωτερικό crystal στο pin P23
    clk_50mhz      : in  std_logic;
    
    -- Control signals
    rst_button     : in  std_logic;    -- Συγχρονισμένο reset
    step_btn       : in  std_logic;    -- Κουμπί step
    dir_sw         : in  std_logic;    -- Διακόπτης κατεύθυνσης
    limit_switches : in  std_logic_vector(3 downto 0);  -- Limit switches
    
    -- One-hot outputs για τους 4 κινητήρες
    motor_enable   : out std_logic_vector(3 downto 0);
    
    -- PWM outputs για velocity control
    pwm_outputs    : out std_logic_vector(3 downto 0);
    
    -- Debug outputs
    debug_led      : out std_logic_vector(7 downto 0);
    
    -- Position feedback
    position_out   : out std_logic_vector(15 downto 0)
  );
end entity RoboticArmFourDof;

architecture complete of RoboticArmFourDof is
  -- Σήματα για interconnection
  signal step_debounced     : std_logic;
  signal step_sync          : std_logic;
  signal step_prev          : std_logic;
  signal internal_rst       : std_logic;
  signal position_count     : unsigned(15 downto 0);
  signal motor_enable_int   : std_logic_vector(3 downto 0);
  signal pwm_counter        : unsigned(7 downto 0) := (others => '0');
  signal pwm_duty_cycle     : unsigned(7 downto 0) := to_unsigned(128, 8); -- 50% duty cycle
  
  -- Σήματα για debouncer
  signal debounce_count     : integer range 0 to 100000 := 0;
  signal button_prev        : std_logic := '0';
  
  -- Σήματα για synchronized reset
  signal rst_sync           : std_logic_vector(2 downto 0) := (others => '0');
  
begin

  -- 1. Synchronized Reset Process
  sync_reset_process: process(clk_50mhz)
  begin
    if rising_edge(clk_50mhz) then
      rst_sync <= rst_sync(1 downto 0) & rst_button;
      internal_rst <= rst_sync(2);
    end if;
  end process;

  -- 2. Debouncer Process
  debouncer_process: process(clk_50mhz)
  begin
    if rising_edge(clk_50mhz) then
      button_prev <= step_btn;
      if button_prev /= step_btn then
        debounce_count <= 0;
      elsif debounce_count < 100000 then
        debounce_count <= debounce_count + 1;
      else
        step_debounced <= button_prev;
      end if;
    end if;
  end process;

  -- 3. Position Counter Process
  position_counter_process: process(clk_50mhz, internal_rst)
  begin
    if internal_rst = '1' then
      position_count <= (others => '0');
    elsif rising_edge(clk_50mhz) then
      if step_debounced = '1' then
        if dir_sw = '1' then
          position_count <= position_count + 1;
        else
          position_count <= position_count - 1;
        end if;
      end if;
    end if;
  end process;

  -- 4. Main State Machine Process
  state_machine_process: process(clk_50mhz, internal_rst)
  begin
    if internal_rst = '1' then
      motor_enable_int <= "0001";  -- Αρχική κατάσταση
    elsif rising_edge(clk_50mhz) then
      if step_debounced = '1' then
        -- Έλεγχος limit switches
        if not((dir_sw = '1' and limit_switches(3) = '1' and motor_enable_int(3) = '1') or
               (dir_sw = '0' and limit_switches(0) = '1' and motor_enable_int(0) = '1')) then
          
          if dir_sw = '1' then
            motor_enable_int <= motor_enable_int(2 downto 0) & motor_enable_int(3);
          else
            motor_enable_int <= motor_enable_int(0) & motor_enable_int(3 downto 1);
          end if;
        end if;
      end if;
    end if;
  end process;

  -- 5. PWM Generator Process
  pwm_process: process(clk_50mhz)
  begin
    if rising_edge(clk_50mhz) then
      pwm_counter <= pwm_counter + 1;
      
      for i in 0 to 3 loop
        if pwm_counter < pwm_duty_cycle and motor_enable_int(i) = '1' then
          pwm_outputs(i) <= '1';
        else
          pwm_outputs(i) <= '0';
        end if;
      end loop;
    end if;
  end process;

  -- 6. Output Assignments
  motor_enable <= motor_enable_int;
  position_out <= std_logic_vector(position_count);
  
  -- Debug outputs
  debug_led(3 downto 0) <= motor_enable_int;
  debug_led(7 downto 4) <= std_logic_vector(position_count(3 downto 0));

end architecture complete;
