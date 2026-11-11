using SW2URDF.URDFExport;
using System.Windows.Forms;
using Xunit;

namespace SW2URDF.Test
{
    /// <summary>
    /// Keys pressed in the link tree of the PropertyManager page. A new assembly has no selected
    /// or right-clicked node yet; keys used to throw a NullReferenceException then.
    /// Does not need SolidWorks.
    /// </summary>
    public class TestTreeKeyActions
    {
        [Theory]
        [InlineData(Keys.A)]
        [InlineData(Keys.Delete)]
        [InlineData(Keys.F2)]
        [InlineData(Keys.Enter)]
        [InlineData(Keys.Escape)]
        [InlineData(Keys.Control | Keys.A)]
        [InlineData(Keys.F)]
        public void WithoutANodeNoKeyDoesAnything(Keys key)
        {
            Assert.Equal(TreeKeyAction.None, TreeKeyActions.Decide(false, false, key));
            Assert.Equal(TreeKeyAction.None, TreeKeyActions.Decide(false, true, key));
        }

        [Fact]
        public void EnterAndEscapeFinishAnEditInProgress()
        {
            Assert.Equal(TreeKeyAction.AcceptEdit, TreeKeyActions.Decide(true, true, Keys.Enter));
            Assert.Equal(TreeKeyAction.CancelEdit, TreeKeyActions.Decide(true, true, Keys.Escape));
        }

        [Fact]
        public void OtherKeysAndNoEditDoNothing()
        {
            Assert.Equal(TreeKeyAction.None, TreeKeyActions.Decide(true, true, Keys.A));
            Assert.Equal(TreeKeyAction.None, TreeKeyActions.Decide(true, false, Keys.Enter));
            Assert.Equal(TreeKeyAction.None, TreeKeyActions.Decide(true, false, Keys.Escape));
        }

        [Fact]
        public void OnlyANodeWithAParentCanBeRemoved()
        {
            Assert.False(TreeKeyActions.CanRemove(false, false));
            Assert.False(TreeKeyActions.CanRemove(true, false));
            Assert.True(TreeKeyActions.CanRemove(true, true));
        }
    }
}