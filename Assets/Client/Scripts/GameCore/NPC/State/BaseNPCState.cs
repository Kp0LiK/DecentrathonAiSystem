using CustomTools.Updater;


namespace Client
{
    public abstract class BaseNPCState : IUpdateMono
    {
        protected INPCStateSwitcher StateSwitcher;

        protected BaseNPCState(INPCStateSwitcher stateSwitcher)
        {
            StateSwitcher = stateSwitcher;
        }

        public abstract void StartState();
        protected abstract void Action();
        public abstract void EndState();

        public void Tick()
        {
            Action();
        }
    }
}