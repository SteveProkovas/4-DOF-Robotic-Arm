library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RoboticArmFourDof is
  port (
    -- Clock: 50MHz από εσωτερικό crystal στο pin P23 (σύμφωνα με την τεκμηρίωση)
    clk_50mhz  : in  std_logic;
    
    -- Control signals (θα συνδεθούν σε κουμπιά/διακόπτες)
    rst_button : in  std_logic;    -- Συγχρονισμένο reset
    step_btn   : in  std_logic;    -- Κουμπί step
    dir_sw     : in  std_logic;    -- Διακόπτης κατεύθυνσης
    
    -- One-hot outputs για τους 4 κινητήρες
    motor_enable : out std_logic_vector(3 downto 0)
  );
end entity RoboticArmFourDof;

architecture rtl of RoboticArmFourDof is
  signal r_q : std_logic_vector(3 downto 0) := "0001"; -- αρχική κατάσταση
  signal step_debounced : std_logic := '0';
  signal step_prev : std_logic := '0';
begin

  -- Απλός debouncer για το κουμπί step
  process(clk_50mhz)
  begin
    if rising_edge(clk_50mhz) then
      step_prev <= step_btn;
      if step_btn = '1' and step_prev = '0' then
        step_debounced <= '1';
      else
        step_debounced <= '0';
      end if;
    end if;
  end process;

  -- Κύρια λογική ring counter
  process(clk_50mhz)
  begin
    if rising_edge(clk_50mhz) then
      if rst_button = '1' then
        r_q <= "0001"; -- Joint0 active
      elsif step_debounced = '1' then
        if dir_sw = '1' then
          -- rotate left: {q2,q1,q0,q3}
          r_q <= r_q(2 downto 0) & r_q(3);
        else
          -- rotate right: {q0,q3,q2,q1}
          r_q <= r_q(0) & r_q(3 downto 1);
        end if;
      end if;
    end if;
  end process;

  motor_enable <= r_q;

end architecture rtl;
